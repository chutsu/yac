#include <gtest/gtest.h>
#include "Sim.hpp"

namespace yac {

TEST(Sim, construct) {
  Sim sim;
  sim.save("/tmp/sim.csv");
}

} // namespace yac
