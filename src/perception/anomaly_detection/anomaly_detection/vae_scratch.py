import os
import cv2
import torch
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from torch import nn, Tensor
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from PIL import Image as PILImage
import torch.nn.functional as F

#1. ------ model definition ------
class VAE(nn.Module):
    def __init__(self, 
    in_channels: int = 3, 
    latent_dim: int = 64, 
    hidden_dims= List[int] = None,
    img_size: int = 256):

        super(VAE, self).__init__()

        if hidden_dims is None:
            hidden_dims = [32, 64, 128, 256, 512]

        self.latent_dim = latent_dim
        self.img_size = img_size

        # ---- Encoder ----
        modules = []
        for h_dim in hidden_dims:
            modules.append(nn.Sequential(
                nn.Conv2d(in_channels, h_dim, 3, 2, 1),
                nn.BatchNorm2d(h_dim),
                nn.LeakyReLU())
            )
            in_channels = h_dim
        self.encoder_layers = nn.Sequential(*modules)
        final_encoder_dim = img_size // (2 ** len(hidden_dims))
        self.flatten_dim = hidden_dims[-1] * final_encoder_dim * final_encoder_dim #8 * 8 if input is 256, 256
        self.fc_mu = nn.Linear(self.flatten_dim, latent_dim)
        self.fc_logvar = nn.Linear(self.flatten_dim, latent_dim)
        
        #---- Decoder ----
        self.decoder_input = nn.Linear(latent_dim, self.flatten_dim)

        decoder_hidden_dims = hidden_dims[::-1]
        modules = []
        for i in range(len(decoder_hidden_dims) - 1):
            modules.append(nn.Sequential(
                nn.ConvTranspose2d(decoder_hidden_dims[i], decoder_hidden_dims[i + 1], 3, 2, 1, 1),
                nn.BatchNorm2d(decoder_hidden_dims[i + 1]),
                nn.LeakyReLU())
            )
        self.decoder_layers = nn.Sequential(*modules)
        self.final_layer = nn.Sequential(
            nn.ConvTranspose2d(decoder_hidden_dims[-1], decoder_hidden_dims[-1], 3, 2, 1, 1),
            nn.BatchNorm2d(decoder_hidden_dims[-1]),
            nn.LeakyReLU(),
            nn.Conv2d(decoder_hidden_dims[-1], out_channels=3, kernel_size=3, padding=1),
            nn.Tanh()) #images must be normalized to [-1, 1]


    def encode(self, x: Tensor) -> List[Tensor]:
        """
        Encodes the input by passing through the encoder network
        and returns the latent codes.
        :param input: (Tensor) Input tensor to encoder [N x C x H x W]
        :return: (Tensor) List of latent codes
        """
        x = self.encoder_layers(x)
        x = torch.flatten(x, start_dim=1)
        mu = self.fc_mu(x)
        logvar = self.fc_logvar(x)
        return mu, logvar
    
    def decode(self, z: Tensor) -> Tensor:
        """
        Maps the given latent codes
        onto the image space.
        :param z: (Tensor) [B x D]
        :return: (Tensor) [B x C x H x W]
        """
        x = self.decoder_input(z)
        final_encoder_dim = self.img_size // (2 ** len(self.encoder_layers))
        x = x.view(-1, self.decoder_input.out_features // (final_encoder_dim * final_encoder_dim), final_encoder_dim, final_encoder_dim)
        x = self.decoder_layers(x)
        x = self.final_layer(x)
        return x

    def reparameterize(self, mu: Tensor, logvar: Tensor) -> Tensor:
        """
        Reparameterization trick to sample from N(mu, var) from N(0,1).
        :param mu: (Tensor) Mean of the latent Gaussian [B x D]
        :param logvar: (Tensor) Log Variance of the latent Gaussian [B x D]
        :return: (Tensor) Sample Latent Vector [B x D]
        """
        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)
        return eps * std + mu
    
    def forward(self, x: Tensor, **kwargs) -> List[Tensor]:
        mu, logvar = self.encode(x)
        z = self.reparameterize(mu, logvar)
        recon = self.decode(z)
        return recon, x, mu, logvar

#2.------- loss function definition ------      
def vae_loss_function(recon, input, mu, logvar, kld_weight=1.0): #weight might need to be tuned
    """
    Calculates the VAE loss, which is a combination of reconstruction loss and KLD.
    :param recon: (Tensor) Reconstructed input
    :param input: (Tensor) Original input
    :param mu: (Tensor) Mean of the latent distribution
    :param logvar: (Tensor) Log-variance of the latent distribution
    :param kld_weight: (float) Weight for the KLD term
    :return: (List[Tensor]) Total VAE loss, Reconstruction loss (detached), KLD loss (detached)
    """

    recon_loss = F.mse_loss(recon, input, reduction = 'sum') #MSE (esp is pixels normalized)
    kld = torch.mean(-0.5 * torch.sum(1 + logvar - mu.pow(2) - logvar.exp(), dim=1), dim=0)

    # For consistent total loss reporting, often recon_loss is averaged per pixel/batch
    # Let's adjust for batch size for logging total loss per sample
    batch_size = input.size(0)
    recon_loss_per_sample = recon_loss / batch_size
    
    total_loss_of_recon_persample = recon_loss_per_sample + kld_weight * kld
    return total_loss_of_recon_persample, recon_loss_per_sample.detach(), recon_loss + kld_weight * kld, recon_loss.detach(), kld.detach()

    #for the recon loss, i am not sure whether to use per sample or not, so for now i am returning both per sample and non-modified
