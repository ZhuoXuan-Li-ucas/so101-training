"""Check source geometry, temporal alignment and real model hook transparency."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from visualize_act_attention import AttentionCapture, current_index, nearest_frame, split_attention


class AttentionGeometryTests(unittest.TestCase):
    def test_two_non_square_camera_maps_keep_order_and_probability_mass(self):
        # Distinct spatial peaks expose transposition, camera swapping, and prefix errors.
        weights = np.zeros((2, 14), dtype=np.float32)
        weights[:, :2] = [.1, .2]
        weights[0, 2 + 5], weights[0, 8 + 1] = .4, .3
        weights[1, 2 + 0], weights[1, 8 + 5] = .4, .3
        first, second = split_attention(weights, [(2, 3), (3, 2)], 2)
        self.assertAlmostEqual(float(first[0, 1, 2]), .4)
        self.assertAlmostEqual(float(second[0, 0, 1]), .3)
        self.assertAlmostEqual(float(second[1, 2, 1]), .3)
        self.assertAlmostEqual(float(first.sum(1).sum(1)[1]), .4)

    def test_bad_geometry_or_probabilities_fail(self):
        for weights in (np.zeros((1, 14)), np.full((1, 13), 1 / 13)):
            with self.assertRaises(ValueError):
                split_attention(weights, [(2, 3), (3, 2)], 2)


class TimelineTests(unittest.TestCase):
    def test_nearest_input_reports_missing_frame_instead_of_hiding_it(self):
        self.assertEqual(nearest_frame([1., 2., 3.], 2.), (1, 0.))
        index, offset = nearest_frame([1., 2., 3.], .96)
        self.assertEqual(index, 0)
        self.assertAlmostEqual(offset, .04)
        with self.assertRaises(ValueError):
            nearest_frame([1., 2.], None)

    def test_dispatch_does_not_use_future_chunk_during_inference(self):
        # Chunk two's observation may have been captured at 3.0 s, but its first
        # action only finishes dispatch at 3.2 s. Keep chunk one until then.
        send_times = [.28, .32, 3.2]
        self.assertEqual(current_index(send_times, .27), -1)
        self.assertEqual(current_index(send_times, 3.15), 1)
        self.assertEqual(current_index(send_times, 3.2), 2)


class RealACTTests(unittest.TestCase):
    def test_capture_preserves_predictions_and_non_square_backbone_geometry(self):
        import torch
        from lerobot.configs.types import FeatureType, PolicyFeature
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.policies.act.modeling_act import ACTPolicy

        torch.set_num_threads(2)
        torch.manual_seed(3)
        config = ACTConfig(device="cpu", pretrained_backbone_weights=None, chunk_size=3,
                           n_action_steps=3, dim_model=32, n_heads=4, dim_feedforward=64,
                           n_encoder_layers=1, n_decoder_layers=1, n_vae_encoder_layers=1,
                           input_features={"observation.state": PolicyFeature(FeatureType.STATE, (6,)),
                                           "observation.images.top": PolicyFeature(FeatureType.VISUAL, (3, 64, 96)),
                                           "observation.images.wrist": PolicyFeature(FeatureType.VISUAL, (3, 64, 96))},
                           output_features={"action": PolicyFeature(FeatureType.ACTION, (6,))})
        policy = ACTPolicy(config).eval()
        batch = {"observation.state": torch.zeros(1, 6),
                 "observation.images.top": torch.rand(1, 3, 64, 96),
                 "observation.images.wrist": torch.rand(1, 3, 64, 96)}
        with torch.inference_mode():
            original = policy.predict_action_chunk(batch)
            with AttentionCapture(policy.model) as capture:
                captured = policy.predict_action_chunk(batch)
            torch.testing.assert_close(original, captured, rtol=0, atol=0)
            self.assertEqual(capture.grids, [(2, 3), (2, 3)])
            self.assertEqual(capture.weights[0].shape, (1, 3, 14))
            split_attention(capture.weights[0][0], capture.grids, 2)
            self.assertFalse(policy.model.decoder.layers[-1].multihead_attn._forward_hooks)


if __name__ == "__main__":
    unittest.main()
