import random
import pickle


def read_scores_data(log_file, cie):
    with open(log_file, mode='rb') as f:
        map_name = pickle.load(f)
        cie.debug('Computing score for: {}'.format(map_name))
        eof = False
        while not eof:
            try:
                position = pickle.load(f)
                print(position)
            except EOFError:
                eof = True

    return None, {
        'lf': random.uniform(0, 100),  # let the God of Randomness decide who's the first on the leaderboard
    }


if __name__ == '__main__':
    print read_scores_data('log.pickle')
