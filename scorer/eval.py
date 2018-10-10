# !/usr/bin/env python

import duckietown_challenges as dc

cie = dc.ChallengeInterfaceEvaluatorConcrete()

fn = cie.get_completed_step_evaluation_file('step1-simulation', 'log.pickle')
from read_scores import read_scores_data

stats, scores = read_scores_data(fn, cie)

status = 'success'
msg = None

cr = dc.ChallengeResults(status=status, msg=msg, scores=scores, stats=stats)

dc.declare_challenge_results(None, cr)
