import os

from duckietown_utils import d8n_make_video_from_bag
from duckietown_utils.bag_visualization import NotEnoughFramesInSlice
import tempfile

def make_video_bag(bag_filename, tmp_dir='.'):
    filename = tempfile.mktemp(".mp4", dir=tmp_dir)
    # out = os.path.join(tmp_dir, os.path.basename(bag_filename) + '.mp4')
    out = filename
    print('%s -> %s' % (bag_filename, out))
    try:
        d8n_make_video_from_bag(bag_filename, '/gym/observations', out)
    except NotEnoughFramesInSlice:
        msg = 'Not enough for a video'
        print(msg)
        return None
    return out


if __name__ == '__main__':
    make_video_bag('example.bag')
