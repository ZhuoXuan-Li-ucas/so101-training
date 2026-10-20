"""Loss aggregation must remain correct for padding, episode boundaries and short batches."""

import importlib.util
from pathlib import Path
import unittest


spec = importlib.util.spec_from_file_location(
    "evaluate_act_full", Path(__file__).resolve().parents[1] / "evaluate_act_full.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
LossAccumulator = module.LossAccumulator


class EvaluationAggregationTests(unittest.TestCase):
    def test_mixed_episode_batch_and_short_tail_have_distinct_weightings(self):
        metrics = LossAccumulator()
        metrics.add(1.0, [45, 45, 45, 46], [4.0, 4.0, 4.0, 4.0], [4, 4, 4, 4])
        metrics.add(3.0, [46], [6.0], [2])
        result = metrics.result()
        self.assertEqual(result["frames"], 5)
        self.assertEqual(result["mean_batch_loss_like_training_log"], 2.0)
        self.assertAlmostEqual(result["valid_action_element_weighted_loss"], 22 / 18)
        self.assertEqual(result["per_episode"]["45"]["frames"], 3)
        self.assertEqual(result["per_episode"]["45"]["valid_action_element_weighted_loss"], 1.0)
        self.assertAlmostEqual(result["per_episode"]["46"]["valid_action_element_weighted_loss"], 10 / 6)
        self.assertEqual(result["per_episode"]["46"]["mean_sample_loss"], 2.0)


    def test_prefix_500_metric_does_not_replace_full_validation_metric(self):
        metrics = LossAccumulator()
        for _ in range(125):
            metrics.add(1.0, [45] * 4, [6.0] * 4, [6] * 4)
        metrics.add(4.0, [46] * 4, [24.0] * 4, [6] * 4)
        result = metrics.result()
        self.assertEqual(result["frames"], 504)
        self.assertEqual(result["prefix_500_batch_mean"], 1.0)
        self.assertAlmostEqual(result["mean_batch_loss_like_training_log"], 129 / 126)
        self.assertEqual(result["per_episode"]["46"]["frames"], 4)


    def test_invalid_evaluation_cannot_receive_a_checkpoint_rank(self):
        for loss, errors, counts in [
            (float("nan"), [1.0], [6]),
            (1.0, [float("inf")], [6]),
            (1.0, [1.0], [0]),
        ]:
            with self.subTest(loss=loss, errors=errors, counts=counts):
                with self.assertRaises(ValueError):
                    LossAccumulator().add(loss, [45], errors, counts)


if __name__ == "__main__":
    unittest.main()
