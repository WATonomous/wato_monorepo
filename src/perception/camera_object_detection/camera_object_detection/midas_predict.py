import cv2
import torch
import os

torch.cuda.empty_cache()
model_type = "DPT_Large" # this one uses dpt_large_384.pt. It is 61 fps.
midas = torch.hub.load("intel-isl/MiDaS", model_type)

# if no cuda memory just turn use cpu
device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
midas.to(device)
midas.eval()

midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")

input_dir = './input'
output_dir = './output'
os.makedirs(output_dir, exist_ok=True)

for filename in os.listdir(input_dir):
    if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
        # Read and transform image
        img = cv2.imread(os.path.join(input_dir, filename))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        input_batch = midas_transforms.dpt_transform(img).to(device)

        with torch.no_grad():
            # Make prediction
            prediction = midas(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=img.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()
        
        # Save image
        output = prediction.cpu().numpy()
        output = cv2.normalize(output, None, 0, 255, cv2.NORM_MINMAX)
        output = cv2.applyColorMap(output.astype('uint8'), cv2.COLORMAP_INFERNO) # gives the image a color map

        cv2.imwrite(os.path.join(output_dir, f"{os.path.splitext(filename)[0]}_depth.jpg"), output)
