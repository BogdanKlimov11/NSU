import numpy as np


def binary_classification_metrics(prediction, ground_truth):
    TP = np.sum([a and b for a, b in zip(prediction, ground_truth)])
    TN = np.sum([not a and not b for a, b in zip(prediction, ground_truth)])
    FP = np.sum([a and not b for a, b in zip(prediction, ground_truth)])
    FN = np.sum([not a and b for a, b in zip(prediction, ground_truth)])

    accuracy = (TP + TN) / len(ground_truth)
    precision = TP / (TP + FP)
    recall = TP / (TP + FN)
    f1 = 2 * precision * recall / (precision + recall)

    return precision, recall, f1, accuracy


def multiclass_accuracy(prediction, ground_truth):
    return np.where(prediction == ground_truth)[0].shape[0] / prediction.shape[0]