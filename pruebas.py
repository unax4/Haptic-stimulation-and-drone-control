import cv2
import numpy as np
import glob
import os

def temporal_median_filter(image_pattern, output_path):
    # Grab all frames matching the pattern and sort them
    image_paths = sorted(glob.glob(image_pattern))
    
    if len(image_paths) < 3:
        print("You need at least 3 frames for a temporal median to work effectively.")
        return

    print(f"Found {len(image_paths)} frames. Processing...")

    frames = []
    for path in image_paths:
        img = cv2.imread(path)
        if img is not None:
            frames.append(img)
    
    if not frames:
        print("Could not read the images.")
        return

    # Stack the frames along a new time axis (Depth)
    # This creates a 4D array: (frame_number, height, width, color_channel)
    stacked_frames = np.stack(frames, axis=0)

    # Calculate the median pixel value across the time axis (axis=0)
    # Outlier pixels (the bright glitches) get discarded, leaving the true background
    median_frame = np.median(stacked_frames, axis=0).astype(np.uint8)

    cv2.imwrite(output_path, median_frame)
    print(f"Success! Saved reconstructed image to: {output_path}")

# Run the function on your specific frame sequence
# Update the pattern if your files are named differently
temporal_median_filter('C:\\Users\\Unax\\Desktop\\Legacy Code\\drone_k417\\Original project\\captures\\reconstructed\\jpeg_carved\\frame_00037.jpg', 'fixed_temporal_reconstruction.jpg')
