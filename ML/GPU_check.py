import tensorflow as tf

print("TF:", tf.__version__)
print("GPUs:", tf.config.list_physical_devices('GPU'))

with tf.device('/GPU:0'):
    x = tf.random.normal((2000, 2000))
    for _ in range(20):
        x = tf.matmul(x, x)
print("Done matmul on GPU")