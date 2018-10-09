import math
import pickle

def read_scores_data(filename):
    log = pickle.load(open(filename))
    print log

    stats = {}
    stats['steps'] = len(lines)

    scores = {}
    scores['score'] = 1


    return stats, scores


if __name__ == '__main__':
    print read_scores_data('log.pickle')
