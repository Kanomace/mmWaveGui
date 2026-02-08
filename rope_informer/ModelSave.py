import numpy as np
import tensorflow as tf
from tensorflow.keras import layers

# 加载数据
def load_data(folder_path):
    data = []
    for i in range(1, 100):
        file_path = f"{folder_path}/pHistBytes_clustered_voxel_{i}.npy"
        frame_data = np.load(file_path)
        data.append(frame_data)
    return np.array(data)

# 加载训练数据
squat_data = load_data("C:/Users/Kano/Desktop/radar_toolbox_1_30_01_03/tools/visualizers/Industrial_Visualizer/binData/trainData/squat/pHistBytes_clustered_voxel")
stand_data = load_data("C:/Users/Kano/Desktop/radar_toolbox_1_30_01_03/tools/visualizers/Industrial_Visualizer/binData/trainData/stand/pHistBytes_clustered_voxel")
walk_data = load_data("C:/Users/Kano/Desktop/radar_toolbox_1_30_01_03/tools/visualizers/Industrial_Visualizer/binData/trainData/walk/pHistBytes_clustered_voxel")

# 创建标签
squat_labels = np.zeros(len(squat_data))
stand_labels = np.ones(len(stand_data))
walk_labels = np.ones(len(walk_data)) * 2

# 数据拆分为时间窗口
def create_time_windows(data, window_size):
    windows = []
    for i in range(len(data) - window_size + 1):
        window = data[i:i+window_size]
        windows.append(window)
    return np.array(windows)

window_size = 3
squat_windows = create_time_windows(squat_data, window_size)
stand_windows = create_time_windows(stand_data, window_size)
walk_windows = create_time_windows(walk_data, window_size)

# 合并数据和标签
X = np.concatenate((squat_windows, stand_windows, walk_windows), axis=0)
y = np.concatenate((squat_labels, stand_labels, walk_labels), axis=0)

# 将标签转换为独热编码
y = tf.keras.utils.to_categorical(y)

# 数据集划分为训练集、验证集和测试集
train_ratio = 0.7
val_ratio = 0.15
test_ratio = 0.15

num_samples = len(X)
num_train = int(train_ratio * num_samples)
num_val = int(val_ratio * num_samples)

indices = np.random.permutation(num_samples)
train_indices = indices[:num_train]
val_indices = indices[num_train:num_train+num_val]
test_indices = indices[num_train+num_val:]

X_train, y_train = X[train_indices], y[train_indices]
X_val, y_val = X[val_indices], y[val_indices]
X_test, y_test = X[test_indices], y[test_indices]

# 构建3D CNN模型
model = tf.keras.Sequential()
model.add(layers.Conv3D(32, kernel_size=(3, 3, 3), activation='relu', input_shape=(window_size, 50, 25, 50)))
model.add(layers.Flatten())
model.add(layers.Dense(64, activation='relu'))
model.add(layers.Dense(3, activation='softmax'))

# 编译模型
model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])

# 训练模型
history = model.fit(X_train, y_train, validation_data=(X_val, y_val), batch_size=32, epochs=10)

# 评估模型
test_loss, test_accuracy = model.evaluate(X_test, y_test)
print("Test Loss:", test_loss)
print("Test Accuracy:", test_accuracy)

# 保存模型
save_path = "C:/Users/Kano/Desktop/radar_toolbox_1_30_01_03/tools/visualizers/Industrial_Visualizer/binData/trainData/saved_model/model.keras"
model.save(save_path)