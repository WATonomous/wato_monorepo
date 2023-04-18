## Article followed: https://learnopencv.com/pytorch-for-beginners-image-classification-using-pre-trained-models/

from torchvision import transforms, models
from PIL import Image
import torch 

# Load the pre-trained model
alexnet = models.alexnet(pretrained=True)

# Create a transformer that transforms a given image to fit the parameters of the model
transform = transforms.Compose([
 transforms.Resize(256),
 transforms.CenterCrop(224),
 transforms.ToTensor(),
 transforms.Normalize(
 mean=[0.485, 0.456, 0.406],
 std=[0.229, 0.224, 0.225]
 )])

# Load the image
img = Image.open("test_img.jpg")

# Transform the image
img_t = transform(img)

# Prepare image for the neural network
batch_t = torch.unsqueeze(img_t, 0)

# Set model to evaluation mode
alexnet.eval()

# Send processed image through the neural network
out = alexnet(batch_t)

# Make all labels into a variable
with open('items.txt') as f:
  classes = [line.strip() for line in f.readlines()]

# Store the index in the vector with the highest value
_, index = torch.max(out, 1)

# Convert the value in the vector into a precentage (% accuracy)
percentage = torch.nn.functional.softmax(out, dim=1)[0] * 100

# Print the corresponding label at the index stored and the accuracy that the model predicted with
print(classes[index[0]], percentage[index[0]].item(), "%")
