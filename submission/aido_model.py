import tensorflow as tf
import numpy as np
from .utils import resnet_1


class TensorflowLearningModel:
    def __init__(self, seed=1234):
        # model definition
        self._observation = None
        self._action = None
        self._computation_graph = None
        self._optimizer = None
        self._loss = None
        self._training_step = tf.Variable(0, trainable=False, name='global_step')

        # saving, restoring &  logging
        self.tf_session = tf.InteractiveSession()
        self.tf_checkpoint = None
        self.tf_saver = None
        self.summary_merge = None
        self.summary_writer = None
        self.last_loss = None

        self.seed = seed

    def predict(self, state):
        action = self.tf_session.run([self._computation_graph], feed_dict={
            self._observation: [state],
        })
        return np.squeeze(action)

    def reset(self):
        self.tf_session.run(tf.global_variables_initializer())
        self.tf_session.run(tf.local_variables_initializer())
        tf.train.global_step(self.tf_session, self._training_step)

    def train(self, state, action):
        summary, step, _, learning_loss = self.tf_session.run(
            [self.summary_merge, self._training_step, self._optimizer, self._loss],
            feed_dict={
                self._observation: state,
                self._action: action
            }
        )
        self.summary_writer.add_summary(summary, step)
        self.last_loss = learning_loss
        return learning_loss

    def commit(self):
        self.tf_saver.save(self.tf_session, self.tf_checkpoint, global_step=self._training_step)

    def computation_graph(self):
        model = resnet_1(self._preprocessed_state, seed=self.seed)
        model = tf.layers.dense(model, units=64, activation=tf.nn.relu,
                                kernel_initializer=tf.contrib.layers.xavier_initializer(uniform=False, seed=self.seed),
                                bias_initializer=tf.contrib.layers.xavier_initializer(uniform=False, seed=self.seed))
        model = tf.layers.dense(model, units=32, activation=tf.nn.relu,
                                kernel_initializer=tf.contrib.layers.xavier_initializer(uniform=False, seed=self.seed),
                                bias_initializer=tf.contrib.layers.xavier_initializer(uniform=False, seed=self.seed))

        model = tf.layers.dense(model, self._action.shape[1])

        with tf.name_scope('losses'):
            loss = tf.losses.mean_squared_error(model, self._action)
            tf.summary.scalar('mse', loss)

        return [model], loss

    def initialize(self, input_shape, action_shape, storage_location):
        if not self._computation_graph:
            self._create(input_shape, action_shape)
            self._storing(storage_location, fail_on_empty=True)
            self.training = False

    def prepare_for_train(self, input_shape, output_shape, optimizer, storage_location):
        if not self._computation_graph:
            self._create(input_shape, output_shape)
            self._optimizer = optimizer.minimize(loss=self._loss, global_step=self._training_step)

            self.reset()

            self._logging(storage_location)
            self._storing(storage_location)

            self.training = True

    def _pre_process(self):
        resize = tf.map_fn(lambda frame: tf.image.resize_images(frame, (60, 80)), self._observation)
        and_standardize = tf.map_fn(lambda frame: tf.image.per_image_standardization(frame), resize)
        self._preprocessed_state = and_standardize

    def _create(self, input_shape, output_shape):
        self._observation = tf.placeholder(dtype=tf.float32, shape=input_shape, name='state')
        self._action = tf.placeholder(dtype=tf.float32, shape=output_shape, name='action')
        self._pre_process()

        self._computation_graph, self._loss = self.computation_graph()

    def _logging(self, location):
        self.summary_merge = tf.summary.merge_all()
        self.summary_writer = tf.summary.FileWriter(location, self.tf_session.graph)
        self.last_loss = float('inf')

    def _storing(self, location, fail_on_empty=False):
        self.tf_saver = tf.train.Saver(filename='model', max_to_keep=2)

        self.tf_checkpoint = tf.train.latest_checkpoint(location)
        if self.tf_checkpoint:
            self.tf_saver.restore(self.tf_session, self.tf_checkpoint)
        else:
            if fail_on_empty:
                raise FileNotFoundError()
            else:
                self.tf_checkpoint = location + 'model'

    def close(self):
        self.tf_session.close()

