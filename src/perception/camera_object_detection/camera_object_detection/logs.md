## Mar 7 2024

- Merged the traffic sign dataset with the traffic light dataset
- The merged dataset has 164 labelled images and 386 including the null images
- The model was trained on the 164 image dataset which has an accuracy of ... (determine what metrics to use instead of qualitatively checking on ros bags)
- There currently exist three classes:
    - Green
    - Yellow
    - Red
- However traffic lights can come in many different forms (think red light with a green left turn arrow)
- A possible solution to this is to segment the traffic light into two pieces, one with the top three dots lights (green, yellow and red) and the other with the bottom light (which would be a left/right turn signal etc.)