#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第二轮图像识别与播报服务。

提供 actionlib 接口 /round2_vision/recognize：
  - stage="qr"    : 用机械臂相机读取现场二维码，播报"前往车间获取<任务>"
  - stage="block" : 识别与任务对应的物块，播报"取得<任务>，现在前往对应生产车间。"

设计要点（对应赛事要求）：
  - 全程使用现场相机图像与本地模型，不读取 Gazebo 真值；
  - 单次任务有超时、可被 action 取消；
  - 识别结果、原图、音频全部落盘，供裁判核验。
"""
import json
import os
import subprocess
import tempfile
import threading
import time

import cv2
import numpy as np
import rospy
from actionlib import SimpleActionServer
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Empty, String

from round2_vision.msg import RecognizeTaskAction, RecognizeTaskFeedback, RecognizeTaskResult

# 类别 -> 中文词组（二维码解码结果与贴图文件名都映射到这里）
LABELS = {
    "food": "食品",
    "electronics": "电子产品",
    "daily_necessities": "日用品",
    "食品": "食品",
    "电子产品": "电子产品",
    "日用品": "日用品",
}

# 贴图文件名（不带扩展名）-> 类别键
TEMPLATE_FILES = {
    "food": "Food",
    "electronics": "Electronics",
    "daily_necessities": "Daily_Necessities",
}


class VisionServer(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.latest = None
        self.lock = threading.Lock()

        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.arm_topic = rospy.get_param("~arm_topic", "/arm_control/move_to_pose")
        self.voice = rospy.get_param("~voice", "cmn")
        self.voice_rate = int(rospy.get_param("~voice_rate", 150))
        self.audio_device = rospy.get_param("~audio_device", "plughw:1,0")
        self.play_audio = bool(rospy.get_param("~play_audio", True))
        self.evidence_root = rospy.get_param("~evidence_root", os.path.expanduser("~/vision_evidence"))
        self.min_score = float(rospy.get_param("~min_block_score", 0.50))
        self.min_margin = float(rospy.get_param("~min_block_margin", 0.09))
        self.poses = {
            "qr": rospy.get_param("~qr_poses"),
            "block": rospy.get_param("~block_poses"),
        }
        self.stow_pose = rospy.get_param("~stow_pose")
        self.joint_state = None

        self.arm_pub = rospy.Publisher(self.arm_topic, String, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=10, latch=True)

        self.templates = self._load_templates()
        rospy.Subscriber(self.image_topic, Image, self._on_image, queue_size=1)

        rospy.Subscriber("/joint_states", JointState, self._on_joints, queue_size=1)

        self.server = SimpleActionServer("~recognize", RecognizeTaskAction,
                                         execute_cb=self._execute, auto_start=False)
        self.server.start()
        rospy.Subscriber("~prepare_qr", Empty, self._prepare_qr, queue_size=1)
        self._status({"state": "READY", "templates": sorted(self.templates)})
        rospy.loginfo("round2_vision 就绪, 模板=%s", sorted(self.templates))

    # ---------- 基础设施 ----------
    def _status(self, payload):
        payload = dict(payload, stamp=rospy.Time.now().to_sec())
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        rospy.loginfo("vision status: %s", json.dumps(payload, ensure_ascii=False))

    def _on_image(self, msg):
        with self.lock:
            self.latest = msg

    def _template_dir(self):
        import rospkg
        return os.path.join(rospkg.RosPack().get_path("car3"), "model/cube/meshes")

    def _load_templates(self):
        base = self._template_dir()
        out = {}
        for key, fname in TEMPLATE_FILES.items():
            path = os.path.join(base, fname + ".png")
            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                rospy.logwarn("缺少模板 %s", path)
                continue
            normalized = cv2.equalizeHist(cv2.resize(img, (128, 128), interpolation=cv2.INTER_AREA))
            # Leave room for minor corner and alignment errors in the warped face.
            out[key] = normalized[12:-12, 12:-12].copy()
        return out

    def _on_joints(self, msg):
        with self.lock:
            self.joint_state = msg

    def _check_active(self, deadline):
        if rospy.is_shutdown() or self.server.is_preempt_requested():
            raise RuntimeError("视觉任务已取消")
        if time.monotonic() >= deadline:
            raise RuntimeError("视觉任务超时")

    def _prepare_qr(self, _):
        # The first QR pose stays within the navigation footprint during travel.
        if not self.server.is_active() and self.arm_pub.get_num_connections():
            self.arm_pub.publish(String(data=json.dumps(self.poses["qr"][0])))
            rospy.loginfo("QR camera preposition requested")

    def _command_arm(self, angles, deadline):
        self._check_active(deadline)
        while not self.arm_pub.get_num_connections():
            self._check_active(deadline)
            time.sleep(0.02)
        sent = rospy.Time.now()
        self.arm_pub.publish(String(data=json.dumps(angles)))
        return sent

    def _wait_for_arm(self, angles, sent, deadline):
        names = ["arm_joint%d" % i for i in range(1, 6)]
        stable_since = None
        while True:
            self._check_active(deadline)
            with self.lock:
                state = self.joint_state
            if state is not None and state.header.stamp > sent:
                positions = dict(zip(state.name, state.position))
                settled = all(name in positions and abs(positions[name] - angle) < 0.025
                              for name, angle in zip(names, angles))
                if not settled:
                    stable_since = None
                elif stable_since is None:
                    stable_since = state.header.stamp
                elif (state.header.stamp - stable_since).to_sec() >= 0.12:
                    return state.header.stamp
            time.sleep(0.02)

    def _capture(self, stage, deadline, count):
        for pose_index, angles in enumerate(self.poses[stage]):
            sent = self._command_arm(angles, deadline)
            after = self._wait_for_arm(angles, sent, deadline)
            for frame_index in range(count):
                while True:
                    self._check_active(deadline)
                    with self.lock:
                        msg = self.latest
                    if msg is not None and msg.header.stamp > after:
                        break
                    time.sleep(0.02)
                after = msg.header.stamp
                yield pose_index, frame_index, self.bridge.imgmsg_to_cv2(msg, "bgr8")

    # ---------- 二维码 ----------
    @staticmethod
    def _decode_variants(frame):
        from pyzbar import pyzbar
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        detected, corners = cv2.QRCodeDetector().detect(gray)
        variants = [("gray", gray), ("mirrored", cv2.flip(gray, 1))]
        if detected:
            points = corners.reshape(4, 2).astype(np.float32)
            square = np.array([[0, 0], [251, 0], [251, 251], [0, 251]], dtype=np.float32)
            transform = cv2.getPerspectiveTransform(points, square)
            patch = cv2.warpPerspective(gray, transform, (252, 252))
            patch = cv2.copyMakeBorder(patch, 48, 48, 48, 48, cv2.BORDER_CONSTANT, value=255)
            variants.extend([("rectified", patch), ("rectified_mirrored", cv2.flip(patch, 1))])
            # The supplied boards use a version 1 QR. Resample module centers
            # so foreshortening and Gazebo's texture filtering do not smear edges.
            inner = patch[48:-48, 48:-48]
            for inset in (0, 2, -2, 5):
                centers = ((np.arange(21) + 0.5) * (252 - 2 * inset) / 21 + inset).astype(int)
                samples = inner[np.ix_(centers, centers)]
                _, modules = cv2.threshold(samples, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                canvas = np.pad(modules, 4, constant_values=255).repeat(8, axis=0).repeat(8, axis=1)
                variants.extend([("module_grid", canvas), ("module_grid_mirrored", cv2.flip(canvas, 1))])
        for name, img in variants:
            binary = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
            for source in (img, binary):
                for scale in (1, 2):
                    canvas = source if scale == 1 else cv2.resize(
                        source, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
                    for code in pyzbar.decode(canvas, symbols=[pyzbar.ZBarSymbol.QRCODE]):
                        for encoding in ("utf-8", "gbk"):
                            try:
                                label = code.data.decode(encoding)
                            except UnicodeDecodeError:
                                continue
                            info = {"variant": name, "scale": scale}
                            if label.strip() in LABELS:
                                return label, info
                            # ZBar sometimes guesses Shift-JIS for a UTF-8 QR
                            # without an ECI marker. Undo that conversion.
                            try:
                                recovered = label.encode("shift_jis").decode("utf-8")
                            except UnicodeError:
                                continue
                            if recovered.strip() in LABELS:
                                return recovered, dict(info, zbar_text=label)
        return None, None

    # ---------- 物块 ----------
    @staticmethod
    def _find_candidates(frame):
        """分割受光的方块面，保留中心区域内近似四边形的小面。"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape
        kernel = np.ones((5, 5), np.uint8)
        for threshold in (200, 175, 220):
            mask = cv2.morphologyEx(cv2.inRange(gray, threshold, 255), cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                hull = cv2.convexHull(contour)
                quad = cv2.approxPolyDP(hull, 0.04 * cv2.arcLength(hull, True), True)
                if len(quad) != 4:
                    continue
                x, y, w, h = cv2.boundingRect(quad)
                if (16 <= w <= .32 * width and 16 <= h <= .32 * height
                        and .4 < w / h < 3.4
                        and .12 * width < x < x + w < .88 * width
                        and .12 * height < y < y + h < .83 * height):
                    yield quad

    @staticmethod
    def _warp(quad, frame, size=96):
        pts = quad.reshape(4, 2).astype("float32")
        s = pts.sum(axis=1)
        d = np.diff(pts, axis=1).ravel()
        ordered = np.zeros((4, 2), dtype="float32")
        ordered[0] = pts[np.argmin(s)]
        ordered[2] = pts[np.argmax(s)]
        ordered[1] = pts[np.argmin(d)]
        ordered[3] = pts[np.argmax(d)]
        dst = np.array([[0, 0], [size - 1, 0], [size - 1, size - 1], [0, size - 1]],
                       dtype="float32")
        M = cv2.getPerspectiveTransform(ordered, dst)
        return cv2.warpPerspective(frame, M, (size, size))

    def _match_patch(self, patch_gray):
        """将方块面与三类贴图的四个旋转方向比较。"""
        patch = cv2.resize(patch_gray, (128, 128), interpolation=cv2.INTER_CUBIC)
        patch = cv2.equalizeHist(patch)
        results = []
        for key, tmpl in self.templates.items():
            best = -1.0
            for rot in range(4):
                cand = np.rot90(tmpl, rot)
                score = float(cv2.matchTemplate(patch, cand, cv2.TM_CCOEFF_NORMED).max())
                best = max(best, score)
            results.append((best, key))
        results.sort(reverse=True)
        return results

    def _classify_block(self, frame, evidence_dir, tag):
        cv2.imwrite(os.path.join(evidence_dir, "block_%s.png" % tag), frame)
        for index, quad in enumerate(self._find_candidates(frame)):
            patch = self._warp(quad, frame)
            scores = self._match_patch(cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY))
            cv2.imwrite(os.path.join(evidence_dir, "block_%s_face%02d.png" % (tag, index)), patch)
            yield {"frame": tag, "candidate": index, "scores": scores}

    def _run_audio(self, command, deadline):
        with subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE) as process:
            try:
                while True:
                    self._check_active(deadline)
                    try:
                        _, stderr = process.communicate(timeout=0.1)
                        break
                    except subprocess.TimeoutExpired:
                        continue
                if process.returncode:
                    raise RuntimeError("%s 失败 (%s): %s" %
                                       (command[0], process.returncode, stderr.decode(errors="replace")[:200]))
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()

    def _speak(self, text, evidence_dir, deadline):
        wav = os.path.join(evidence_dir, "broadcast.wav")
        self._run_audio(["espeak-ng", "-v", self.voice, "-s", str(self.voice_rate),
                         "-w", wav, text], deadline)
        if self.play_audio:
            device = ["-D", self.audio_device] if self.audio_device else []
            self._run_audio(["aplay", "-q"] + device + [wav], deadline)
        return {"wav": wav, "played": self.play_audio}

    # ---------- action 主循环 ----------
    def _execute(self, goal):
        started = time.monotonic()
        deadline = started + (goal.timeout if goal.timeout > 0 else 40.0)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        os.makedirs(self.evidence_root, exist_ok=True)
        stage = goal.stage if goal.stage in self.poses else "task"
        evidence_dir = tempfile.mkdtemp(prefix="%s_%s_" % (stage, stamp), dir=self.evidence_root)
        result = RecognizeTaskResult(success=False, evidence_dir=evidence_dir)
        feedback = RecognizeTaskFeedback()

        def tick(text):
            feedback.progress = text
            self.server.publish_feedback(feedback)
            self._status({"state": "RUNNING", "stage": goal.stage, "progress": text})

        try:
            if goal.stage == "qr":
                tick("capture_qr")
                for pose, index, frame in self._capture("qr", deadline, count=2):
                    cv2.imwrite(os.path.join(evidence_dir, "qr_%02d_%02d.png" % (pose, index)), frame)
                    text, info = self._decode_variants(frame)
                    label = LABELS.get(text.strip()) if text else None
                    if label:
                        info["pose"] = pose
                        break
                else:
                    raise RuntimeError("未解出赛事二维码")
                result.task_label = label
                result.utterance = "前往车间获取%s" % label
                result.detail = json.dumps({"qr": text, "info": info, "frame": index},
                                           ensure_ascii=False)
                result.success = True

            elif goal.stage == "block":
                tick("capture_block")
                expected = LABELS.get(goal.expected)
                if not expected:
                    raise RuntimeError("物块任务缺少有效的二维码类别")
                if len(self.templates) != len(TEMPLATE_FILES):
                    raise RuntimeError("物块分类模板不完整")
                matches = {}
                with open(os.path.join(evidence_dir, "block_matches.jsonl"), "w") as evidence:
                    for pose, index, frame in self._capture("block", deadline, count=2):
                        tag = "%02d_%02d" % (pose, index)
                        for candidate in self._classify_block(frame, evidence_dir, tag):
                            self._check_active(deadline)
                            scores = candidate["scores"]
                            score, key = scores[0]
                            margin = score - scores[1][0]
                            print(json.dumps(candidate), file=evidence, flush=True)
                            if score >= self.min_score and margin >= self.min_margin and LABELS[key] == expected:
                                matches[tag] = candidate
                                break
                        if sum(tag.startswith("%02d_" % pose) for tag in matches) >= 2:
                            break
                    else:
                        raise RuntimeError("未能连续确认与二维码对应的物块：%s" % expected)
                label = expected
                result.detail = json.dumps(list(matches.values()), ensure_ascii=False)
                result.task_label = label
                result.utterance = "取得%s，现在前往对应生产车间。" % label
                result.success = True
            else:
                raise RuntimeError("未知 stage: %r" % (goal.stage,))

            tick("broadcast")
            sent = self._command_arm(self.stow_pose, deadline)
            audio = self._speak(result.utterance, evidence_dir, deadline)
            self._wait_for_arm(self.stow_pose, sent, deadline)
            with open(os.path.join(evidence_dir, "result.json"), "w") as fh:
                json.dump({"stage": goal.stage, "expected": goal.expected,
                           "success": result.success, "task_label": result.task_label,
                           "utterance": result.utterance, "detail": result.detail,
                           "audio": audio,
                           "elapsed": round(time.monotonic() - started, 3)},
                          fh, ensure_ascii=False, indent=2)
            self.server.set_succeeded(result)
            self._status({"state": "DONE", "stage": goal.stage,
                          "label": result.task_label, "utterance": result.utterance})

        except Exception as exc:                       # noqa: BLE001
            result.success = False
            result.detail = (result.detail + " | " if result.detail else "") + repr(exc)
            with open(os.path.join(evidence_dir, "result.json"), "w") as fh:
                json.dump({"stage": goal.stage, "expected": goal.expected, "success": False,
                           "detail": result.detail, "elapsed": round(time.monotonic() - started, 3)},
                          fh, ensure_ascii=False, indent=2)
            if self.server.is_preempt_requested():
                self.server.set_preempted(result)
                self._status({"state": "PREEMPTED", "stage": goal.stage})
            else:
                self.server.set_aborted(result)
                self._status({"state": "FAILED", "stage": goal.stage, "detail": result.detail})


def main():
    rospy.init_node("round2_vision")
    VisionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
