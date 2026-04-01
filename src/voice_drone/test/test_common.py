import unittest

from voice_drone.common import canon_target
from voice_drone.common import fallback_parse_intent
from voice_drone.common import target_aliases


class CommonTests(unittest.TestCase):
    def test_canon_target_prefers_exact_canonical_label(self):
        self.assertEqual(canon_target('bottle'), 'bottle')
        self.assertEqual(canon_target('cup'), 'cup')
        self.assertEqual(canon_target('chair'), 'chair')

    def test_aliases_still_map_coke_can_variants(self):
        self.assertEqual(canon_target('coca-cola can'), 'coke can')
        self.assertEqual(canon_target('soda can'), 'soda can')
        self.assertIn('bottle', target_aliases('coke can'))

    def test_fallback_parse_move_and_rotate_commands(self):
        move = fallback_parse_intent('move forward 2')
        self.assertEqual(move['action'], 'move_body')
        self.assertEqual(move['move'], {'x_m': 2.0, 'y_m': 0.0, 'z_m': 0.0})

        rotate = fallback_parse_intent('turn left 45')
        self.assertEqual(rotate['action'], 'rotate')
        self.assertEqual(rotate['yaw_deg'], -45.0)

    def test_fallback_parse_go_to_object_preserves_target(self):
        obj = fallback_parse_intent('track bottle')
        self.assertEqual(obj['action'], 'go_to_object')
        self.assertEqual(obj['target'], 'bottle')


if __name__ == '__main__':
    unittest.main()
