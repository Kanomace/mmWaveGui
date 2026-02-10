import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import pandas as pd
import os


# Define the CNN + LSTM model
class CNNLSTM(nn.Module):
    def __init__(self, input_dim, hidden_dim, num_classes):
        super(CNNLSTM, self).__init__()
        self.cnn = nn.Sequential(
            nn.Conv1d(input_dim, 16, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2, stride=2),
            nn.Conv1d(16, 32, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2, stride=2)
        )
        self.lstm = nn.LSTM(32, hidden_dim, batch_first=True)
        self.fc = nn.Linear(hidden_dim, num_classes)

    def forward(self, x):
        x = self.cnn(x)
        x = x.permute(0, 2, 1)  # Permute dimensions to match LSTM input format
        _, (h, _) = self.lstm(x)
        h = h.squeeze(0)
        out = self.fc(h)
        return out


# Path to the data folder
folder_path = r'C:\Users\Kano\Desktop\radar_toolbox_1_30_01_03\tools\visualizers\Industrial_Visualizer\binData\trainData'

# Load data and assign labels
data = []
labels = []

# Traverse the folder
for root, dirs, files in os.walk(folder_path):
    for file in files:
        if file.endswith(".xlsx"):
            file_path = os.path.join(root, file)
            label = os.path.basename(os.path.dirname(file_path))

            # Read the Excel file
            df = pd.read_excel(file_path)

            # Append data to the list
            data.append(df.values)
            labels.append(label)

# Convert data and labels to NumPy arrays
data = np.array(data)
labels = np.array(labels)

# Data preprocessing
X = np.transpose(data, (0, 2, 1))  # Permute dimensions to match CNN input format
y = labels.reshape(-1, 1)
X = torch.from_numpy(X).float()
y = torch.from_numpy(y).long()

# Split the dataset into training and validation sets (can be adjusted as needed)
train_ratio = 0.8
train_size = int(train_ratio * len(X))
train_X, val_X = X[:train_size], X[train_size:]
train_y, val_y = y[:train_size], y[train_size:]

# Create model instance
input_dim = X.shape[2]
hidden_dim = 64
num_classes = len(np.unique(y))
model = CNNLSTM(input_dim, hidden_dim, num_classes)

# Define loss function and optimizer
criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# Train the model
num_epochs = 10
batch_size = 32

for epoch in range(num_epochs):
    model.train()
    train_loss = 0.0
    train_correct = 0
    train_total = 0

    for i in range(0, len(train_X), batch_size):
        inputs = train_X[i:i + batch_size]
        labels = train_y[i:i + batch_size].squeeze()

        optimizer.zero_grad()

        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

        train_loss += loss.item() * inputs.size(0)
        _, predicted = torch.max(outputs.data, 1)
        train_total += labels.size(0)
        train_correct += (predicted == labels).sum().item()

    train_accuracy = 100 * train_correct / train_total
    train_loss /= len(train_X)

    # Evaluate the model on the validation set
    model.eval()
    val_loss = 0.0
    val_correct = 0
    val_total = 0

    with torch.no_grad():
        for i in range(0, len(val_X), batch_size):
            inputs = val_X[i:i + batch_size]
            labels = val_y[i:i + batch_size(0).squeeze()]

            outputs = model(inputs)
            loss = criterion(outputs, labels)

            val_loss += loss.item() * inputs.size(0)
            _, predicted = torch.max(outputs.data, 1)
            val_total += labels.size(0)
            val_correct += (predicted == labels).sum().item()

    val_accuracy = 100 * val_correct / val_total
    val_loss /= len(val_X)

    print(
        f"Epoch [{epoch + 1}/{num_epochs}], "
        f"Train Loss: {train_loss:.4f}, Train Accuracy: {train_accuracy:.2f}%, "
        f"Val Loss: {val_loss:.4f}, Val Accuracy: {val_accuracy:.2f}%"
    )
