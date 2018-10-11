# !/usr/bin/env python
import duckietown_challenges as dc
from read_scores import read_scores_data
from make_video import make_video_bag

cie = dc.ChallengeInterfaceEvaluatorConcrete()

# get bag from previous steps
bag_filename = cie.get_completed_step_evaluation_file('step1-simulation', 'logfile.bag')

# compute stats and scores
stats, scores = read_scores_data(bag_filename, cie)

# create a video
tmp_dir = cie.get_tmp_dir()

mp4 = make_video_bag(bag_filename, tmp_dir=tmp_dir)
cie.set_evaluation_file('video.mp4', mp4)



status = 'success'
msg = None
cr = dc.ChallengeResults(status=status, msg=msg, scores=scores, stats=stats)
dc.declare_challenge_results(None, cr)

# write files
cie.after_score()
