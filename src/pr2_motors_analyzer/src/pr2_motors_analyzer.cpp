#include "pr2_motors_analyzer/pr2_motors_analyzer.h"

using namespace diagnostic_aggregator;
using namespace std;

PLUGINLIB_REGISTER_CLASS(PR2MotorsAnalyzer,
                         diagnostic_aggregator::PR2MotorsAnalyzer,
                         diagnostic_aggregator::Analyzer)

PR2MotorsAnalyzer::PR2MotorsAnalyzer() :
  path_(""), nice_name_("Motors"), power_board_name_(""),
  runstop_hit_(false), has_initialized_(false), has_power_data_(false),
  has_eth_data_(false)
{ }

PR2MotorsAnalyzer::~PR2MotorsAnalyzer() { }

bool PR2MotorsAnalyzer::init(const string base_name, const ros::NodeHandle &n)
{
  // path_ = BASE_NAME/Motors
  path_ = base_name + "/" + nice_name_;

  if (!n.getParam("power_board_name", power_board_name_))
  {
     ROS_ERROR("No power board name was specified in PR2MotorsAnalyzer! Power board must be \"Power board 10XX\". Namespace: %s", n.getNamespace().c_str());
     return false;
  }

  // Make a "missing" item for the EtherCAT Master
  boost::shared_ptr<StatusItem> item(new StatusItem("EtherCAT Master"));
  eth_master_item_ = item;

  has_initialized_ = true;

  return true;
}

bool PR2MotorsAnalyzer::match(const std::string name)
{
  if (name == "EtherCAT Master")
    return true;

  return name == power_board_name_;
}

bool PR2MotorsAnalyzer::analyze(const boost::shared_ptr<StatusItem> item)
{
  if (item->getName() == power_board_name_)
  {
    has_power_data_ = true;
    runstop_hit_ = item->getValue("Runstop hit") == "True" || item->getValue("Estop hit") == "True";
    return false; // Won't report this item
  }

  // We know our item is "EtherCAT Master"
  eth_master_item_ = item;
  has_eth_data_ = true;

  return true;
}

vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > PR2MotorsAnalyzer::report()
{
  boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> eth_stat = eth_master_item_->toStatusMsg(path_);

  // If we have power data, and runstop is hit, we'll suppress errors
  if (has_eth_data_ && has_power_data_ && runstop_hit_)
  {
    eth_stat->level = diagnostic_msgs::DiagnosticStatus::OK;
  }

  vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > output;
  output.push_back(eth_stat);

  return output;
}
