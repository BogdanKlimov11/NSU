def binary_classification_metrics(prediction, ground_truth):
    '''
    Computes metrics for binary classification

    Arguments:
    prediction, np array of bool (num_samples) - model predictions
    ground_truth, np array of bool (num_samples) - true labels

    Returns:
    precision, recall, f1, accuracy - classification metrics
    '''
    precision = 0
    recall = 0
    accuracy = 0
    f1 = 0

    # TODO: implement metrics!
    # Some helpful links:
    # https://en.wikipedia.org/wiki/Precision_and_recall
    # https://en.wikipedia.org/wiki/F1_score
    f_n, t_n, t_p, f_p = 0, 0, 0, 0

    size = len(ground_truth)

    for i in range(size):
        if prediction[i] and ground_truth[i]:
            t_p += 1
        if prediction[i] and  not ground_truth[i]:
            f_p += 1
        if not prediction[i] and ground_truth[i]:
            f_n += 1
        if not prediction[i] and not ground_truth[i]:
            t_n += 1

    precision = t_p / (t_p + f_p)
    recall = t_p / (t_p + f_n)
    f1 = 2 * precision * recall / (precision + recall)
    accuracy = (t_p + t_n) / size
    
    return precision, recall, f1, accuracy


def multiclass_accuracy(prediction, ground_truth):
    '''
    Computes metrics for multiclass classification

    Arguments:
    prediction, np array of int (num_samples) - model predictions
    ground_truth, np array of int (num_samples) - true labels
    
    Returns:
    accuracy - ratio of accurate predictions to total samples
    '''
    result_good = 0
    size = len(ground_truth)

    for i in range(size):
        if prediction[i] == ground_truth[i]:
            result_good += 1
    # TODO: Implement computing accuracy
    return result_good/size
