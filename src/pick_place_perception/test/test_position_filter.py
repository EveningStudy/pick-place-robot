from collections import deque
import types
import unittest
import numpy as np
from pick_place_perception.object_localizer import ObjectLocalizer


class PositionFilterTests(unittest.TestCase):
    def detector(self):
        return types.SimpleNamespace(positions=deque(maxlen=7), last_position_stamp=None)

    def test_relocated_target_does_not_keep_pregrasp_median(self):
        node = self.detector()
        for i in range(7):
            ObjectLocalizer.filter_position(node,[.46,.20,.83],i*.05)
        placed = ObjectLocalizer.filter_position(node,[.48,-.40,.87],.4)
        np.testing.assert_allclose(placed,[.48,-.40,.87])
        self.assertEqual(len(node.positions),1)

    def test_stale_observations_expire(self):
        node = self.detector()
        ObjectLocalizer.filter_position(node,[.46,.20,.83],1.0)
        result = ObjectLocalizer.filter_position(node,[.48,.20,.83],2.0)
        np.testing.assert_allclose(result,[.48,.20,.83])

    def test_small_noise_still_smoothed(self):
        node = self.detector()
        for x in [.459,.46,.461]:
            result = ObjectLocalizer.filter_position(node,[x,.20,.83],1.0)
        np.testing.assert_allclose(result,[.46,.20,.83])

    def test_simulation_clock_reset_discards_history(self):
        node = self.detector()
        ObjectLocalizer.filter_position(node,[.46,.20,.83],10.0)
        ObjectLocalizer.filter_position(node,[.47,.20,.83],.1)
        self.assertEqual(len(node.positions),1)


if __name__ == '__main__':
    unittest.main()
