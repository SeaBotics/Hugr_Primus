#include <iostream>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/panel.hpp>

int main(int, char**)
{
  try {
    pluginlib::ClassLoader<rviz_common::Panel> loader("hugr_rviz_panel", "rviz_common::Panel");

    std::cout << "Declared classes:\n";
    for (const auto & c : loader.getDeclaredClasses()) {
      std::cout << "  - " << c << "\n";
    }

    std::cout << "\nCreating: hugr_rviz_panel/HugrLeakPanel\n";
    auto inst = loader.createSharedInstance("hugr_rviz_panel/HugrLeakPanel");
    std::cout << "SUCCESS: created instance\n";
    (void)inst;

  } catch (const pluginlib::PluginlibException & ex) {
    std::cerr << "PLUGINLIB EXCEPTION:\n" << ex.what() << "\n";
    return 2;
  } catch (const std::exception & ex) {
    std::cerr << "STD EXCEPTION:\n" << ex.what() << "\n";
    return 3;
  } catch (...) {
    std::cerr << "UNKNOWN EXCEPTION\n";
    return 4;
  }
  return 0;
}
