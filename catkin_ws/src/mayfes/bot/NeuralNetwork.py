'''
The Neural Network interface for the chatbot.
Use this to train the NN as well as to actually use it.
Based on training.py, which was based on rnn3.py, which was based on
https://gist.github.com/danijar/3f3b547ff68effb03e20c470af22c696
https://danijar.com/variable-sequence-lengths-in-tensorflow/
https://medium.com/@erikhallstrm/using-the-tensorflow-lstm-api-3-7-5f2b97ca6b73#.a6ai9917d
'''
from __future__ import print_function, division
from Parser import Parser, FilesManager
import numpy as np
import tensorflow as tf
import os

botname = "okan"


class NeuralNetwork:
    def __init__(self, botname="okan"):
        self.botname = botname
        self.parser = Parser()
        self.filesmanager = FilesManager()
        self.max_input_length = 100
        self.state_size = 400

    def train(self):
        x, y = self.generateData()
        num_epochs = 300
        # maximum num of chacacters in the training data messages
        max_input_length = x.shape[1]
        input_dimensions = x.shape[2]  # how many types of characters there are
        input_data_num = x.shape[0]  # how many different training data
        backprop_length = max_input_length
        state_size = self.state_size
        # how many types of output classes? (how many responses?)
        num_classes = y.shape[1]
        self.batch_size = 5
        batch_size = self.batch_size

        print("max input length\t", max_input_length)
        print("input dimensions\t", input_dimensions)
        print("state_size\t", state_size)
        print("num classes\t", num_classes)

        self.createModel(backprop_length, input_dimensions,
                         num_classes, state_size)

        with tf.Session() as sess:
            sess.run(tf.global_variables_initializer())
            loss_list = []
            for epoch_idx in range(num_epochs):
                _current_cell_state = np.zeros((batch_size, state_size))
                _current_hidden_state = np.zeros((batch_size, state_size))
                print("epoch\t", epoch_idx)
                for i in range(500):
                    batchX = np.zeros(
                        (batch_size, backprop_length, input_dimensions))
                    batchY = np.zeros((batch_size, num_classes))
                    for j in range(batch_size):
                        r = np.random.randint(input_data_num)
                        batchX[j] = x[r]
                        batchY[j] = y[r]
                    _cross_entropy, _train_step, _current_state, _prediction =\
                        sess.run(
                            [self.cross_entropy, self.train_step,
                                self.current_state, self.prediction],
                            feed_dict={
                                self.batchX_placeholder: batchX,
                                self.batchY_placeholder: batchY,
                                self.cell_state: _current_cell_state,
                                self.hidden_state: _current_cell_state
                            })
                    _current_cell_state, _current_hidden_state = _current_state
                    loss_list.append(_cross_entropy)
                    if i % 100 == 0:
                        print("Step", i, "loss", _cross_entropy)
            print("training finished.")
            self.saver = tf.train.Saver()
            if not os.path.exists("./models"):
                os.makedirs("./models")
            self.saver.save(sess, "./models/"+self.botname+"/model.ckpt")

    def prepare(self):
        backprop_length = self.max_input_length
        input_dimensions = 49  # same as model
        num_classes = 31  # same as model
        state_size = self.state_size  # same as model
        self.createModel(backprop_length, input_dimensions,
                         num_classes, state_size)
        self.sess = tf.Session()
        self.sess.run(tf.global_variables_initializer())
        saver = tf.train.Saver()
        saver.restore(self.sess,  "./models/"+self.botname+"/model.ckpt")

    def feedForward(self, sentence):
        '''
        before doing this, you'll probably have to do prepare() with the same parameters as the trained model.
'''
        vector = self.parser.sentence2vecs(sentence)
        x = np.zeros((1, self.max_input_length, vector.shape[1]))
        for i in range(0, min([self.max_input_length, vector.shape[0]])):
            x[0, i, :] = vector[i, :]
        _current_cell_state_a = np.zeros((1, self.state_size))
        _current_hidden_state_a = np.zeros((1, self.state_size))
        _prediction = self.sess.run(self.prediction, feed_dict={
            self.batchX_placeholder: x,
            self.cell_state: _current_cell_state_a,
            self.hidden_state: _current_hidden_state_a,
        })
        print("prediction:\t", _prediction[0])
        argsort = np.argsort(_prediction[0])[::-1]  # reverse order
        print("rank 1\t", argsort[0],
              "\tprobablity\t", _prediction[0][argsort[0]])
        print("rank 2\t", argsort[1],
              "\tprobablity\t", _prediction[0][argsort[1]])
        print("rank 3\t", argsort[2],
              "\tprobablity\t", _prediction[0][argsort[2]])
        return _prediction

    def length(self, seq):
        used = tf.sign(tf.reduce_max(tf.abs(seq), reduction_indices=2))
        length = tf.reduce_sum(used, reduction_indices=1)
        length = tf.cast(length, tf.int32)
        return length

    def last_relevant(self, output, length_input):
        batch_size = tf.shape(output)[0]
        max_length = int(output.get_shape()[1])
        output_size = int(output.get_shape()[2])
        index = tf.range(0, batch_size) * max_length + (length_input - 1)
        flat = tf.reshape(output, [-1, output_size])
        relevant = tf.gather(flat, index)
        return relevant

    def createModel(self, backprop_length, input_dimensions, num_classes, state_size):
        self.batchX_placeholder = tf.placeholder(
            tf.float32, [None, backprop_length, input_dimensions])
        self.batchY_placeholder = tf.placeholder(
            tf.float32, [None, num_classes])
        self.cell_state = tf.placeholder(
            tf.float32, [None, state_size])
        self.hidden_state = tf.placeholder(
            tf.float32, [None, state_size])
        self.init_state = tf.contrib.rnn.LSTMStateTuple(
            self.cell_state, self.hidden_state)
        self.length_input = self.length(self.batchX_placeholder)
        self.output, self.current_state = tf.nn.dynamic_rnn(tf.contrib.rnn.BasicLSTMCell(
            state_size, state_is_tuple=True), self.batchX_placeholder, dtype=tf.float32, sequence_length=self.length_input)
        self.last = self.last_relevant(self.output, self.length_input)
        self.W2 = tf.Variable(np.random.rand(
            state_size, num_classes), dtype=tf.float32)
        self.b2 = tf.Variable(np.zeros((num_classes)), dtype=tf.float32)

        self.prediction = tf.nn.softmax(
            tf.matmul(self.last, self.W2) + self.b2)
        self.cross_entropy = - \
            tf.reduce_sum(self.batchY_placeholder * tf.log(self.prediction))
        self.learning_rate = 0.003
        self.train_step = tf.train.AdagradOptimizer(
            self.learning_rate).minimize(self.cross_entropy)

    def generateData(self):
        '''
        x is shaped (input_data_num x max length x input dimensions)
        for singlex in x:
        singlex is shaped (max_input_length x input_dimensions)
        it holds info about a single input
        each (input_dimensions)-shaped vector represents a character as a
        one-hot vector.
        But it will be a zero vector if the input is shorter than
        max_input_length.

        y is shaped (input_data_num x input_dimensions)
        It holds as one-hot vectors the correct classification for each input.backprop_length
        '''
        dataset = self.filesmanager.load_dataset(self.botname)
        dataset = [[self.parser.sentence2vecs(
            line[0]), line[1]] for line in dataset]
        lengths = [line[0].shape[0]
                   for line in dataset]  # list of how long each sentences is
        print("lengths of each sentence is ", lengths)
        max_input_length = self.max_input_length
        input_data_num = len(lengths)
        input_dimensions = dataset[0][0].shape[1]
        print("max length\t", max_input_length)
        print("input data num\t", input_data_num)
        print("input dimensions\t", input_dimensions)
        x = np.zeros((input_data_num, max_input_length, input_dimensions))
        for i in range(input_data_num):
            for j in range(min(dataset[i][0].shape[0], max_input_length)):
                x[i, j] = dataset[i][0][j]

        y_values = [line[1] for line in dataset]
        num_classes = max(y_values) + 1  # because aRraYS StaRT aT ZeRo

        y = np.zeros((input_data_num, num_classes))
        for i in range(input_data_num):
            y[i, dataset[i][1]] = 1.0
        return x, y


if __name__ == "__main__":
    train = True
    nn = NeuralNetwork("okan_simple")
    if train:
        nn.train()
    else:
        nn.prepare()
        nn.feedForward("こんにちは！")
