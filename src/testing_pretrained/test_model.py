from torchvision import models
import torch
 
alexnet = models.alexnet(pretrained=True)
	
print(alexnet)