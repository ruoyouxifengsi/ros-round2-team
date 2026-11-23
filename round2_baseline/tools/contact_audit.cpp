// Development-only native transport observer. Never provides navigation inputs.
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <atomic>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <chrono>

static volatile std::sig_atomic_t running = 1;
static std::mutex mutex;
static std::ofstream output;
static unsigned long messages = 0, contacts = 0, events = 0;
static unsigned long wallEvents = 0;
static double lastTime = 0;
static std::set<std::pair<std::string, std::string>> active;

static bool cone(const std::string &name) {
  return name.find("construction_cone_") != std::string::npos ||
         name.find("Construction Cone::") != std::string::npos;
}

static void observe(ConstContactsPtr &msg) {
  std::lock_guard<std::mutex> guard(mutex);
  ++messages;
  lastTime = msg->time().sec() + msg->time().nsec() * 1e-9;
  std::set<std::pair<std::string, std::string>> current;
  for (int i = 0; i < msg->contact_size(); ++i) {
    const auto &c = msg->contact(i);
    ++contacts;
    auto a = c.collision1(), b = c.collision2();
    const bool coneContact = (a.find("car3::") == 0 && cone(b)) ||
                             (b.find("car3::") == 0 && cone(a));
    const bool wallContact = (a.find("car3::") == 0 && b.find("round2_map::") == 0) ||
                             (b.find("car3::") == 0 && a.find("round2_map::") == 0);
    if (!coneContact && !wallContact) continue;
    if (a > b) std::swap(a, b);
    auto pair = std::make_pair(a, b);
    current.insert(pair);
    if (!active.count(pair)) {
      if (coneContact) ++events;
      if (wallContact) ++wallEvents;
      output << "{\"type\":\"contact_begin\",\"sim\":" << lastTime
             << ",\"kind\":" << std::quoted(wallContact ? "wall" : "cone")
             << ",\"a\":" << std::quoted(a) << ",\"b\":" << std::quoted(b) << "}\n";
    }
  }
  active.swap(current);
}

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  output.open(argv[1]);
  if (!output) return 2;
  output << std::setprecision(12);
  gazebo::client::setup();
  auto node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init("round2");
  auto subscriber = node->Subscribe("~/physics/contacts", observe);
  std::signal(SIGINT, [](int) { running = 0; });
  std::signal(SIGTERM, [](int) { running = 0; });
  do {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::lock_guard<std::mutex> guard(mutex);
    output << "{\"type\":\"summary\",\"messages\":" << messages
           << ",\"all_contacts\":" << contacts << ",\"cone_contact_events\":" << events
           << ",\"wall_contact_events\":" << wallEvents
           << ",\"sim\":" << lastTime << "}\n";
    output.flush();
  } while (running);
  subscriber.reset();
  gazebo::client::shutdown();
}
