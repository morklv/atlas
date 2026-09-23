import unittest

import numpy as np

from sentinel.segmentation_evaluation import CLASS_NAMES, confusion_matrix, metrics


class SegmentationEvaluationTests(unittest.TestCase):
    def test_confusion_rows_are_truth_and_columns_are_prediction(self):
        truth = np.array([[0, 1], [1, 2]])
        predicted = np.array([[0, 1], [0, 2]])
        matrix = confusion_matrix(predicted, truth)
        self.assertEqual(matrix[0, 0], 1)
        self.assertEqual(matrix[1, 1], 1)
        self.assertEqual(matrix[1, 0], 1)
        self.assertEqual(matrix[2, 2], 1)

    def test_metrics_show_absent_classes_instead_of_claiming_perfect_scores(self):
        matrix = np.zeros((len(CLASS_NAMES), len(CLASS_NAMES)), dtype=int)
        matrix[0, 0] = 8
        matrix[1, 0] = 2
        result = metrics(matrix)
        self.assertEqual(result["pixel_accuracy"], .8)
        self.assertEqual(result["classes"]["road"]["recall"], 0.0)
        self.assertIsNone(result["classes"]["water"]["iou"])
