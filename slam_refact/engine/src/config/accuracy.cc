#include "config/accuracy.h"

namespace slam_refact::config {
namespace {

AccuracyConfig& Store() {
  static AccuracyConfig cfg;
  return cfg;
}

}  // namespace

void SetAccuracy(const AccuracyConfig& cfg) { Store() = cfg; }

const AccuracyConfig& Accuracy() { return Store(); }

}  // namespace slam_refact::config
