import os
import cv2
import torch
from torch import nn, optim
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from PIL import Image as PILImage
import numpy as np
from vae_scratch import VAE, vae_loss_function

# --- Configuration ---
IMG_SIZE = 256 #model expects 256x256 images
LATENT_DIM = 64
HIDDEN_DIMS = [32, 64, 128, 256, 512]
DATA_ROOT_DIR = 'data/train/normal' # Path to training images
CHECKPOINT_DIR = 'checkpoints'
MODEL_NAME = 'vae_model.pth'
NUM_EPOCHS = 50
BATCH_SIZE = 32
LEARNING_RATE = 1e-3
KLD_WEIGHT = 0.00025 # This might need tuning
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

print(f"Using device: {DEVICE}")