#!/usr/bin/python3

import cv2
import imageio
import os

def video_to_gif(video_path, output_gif, fps=10, resize_factor=0.5, speed=1.0):
    cap = cv2.VideoCapture(video_path)
    frames = []
    
    if not cap.isOpened():
        print("Error: Could not open video.")
        return
    
    frame_rate = cap.get(cv2.CAP_PROP_FPS)  # Get original FPS of video
    frame_interval = max(1, int(frame_rate / (fps * speed)))  # Adjust frame interval based on speed

    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        if frame_count % frame_interval == 0:
            # Resize frame (optional)
            frame = cv2.resize(frame, (0, 0), fx=resize_factor, fy=resize_factor)
            
            # Convert BGR to RGB
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frames.append(frame)
        
        frame_count += 1

    cap.release()

    # Save frames as GIF
    imageio.mimsave(output_gif, frames, fps=fps)
    print(f"GIF saved as {output_gif}")

# Example usage
video_path = "/home/peeradon/Videos/Screencasts/Screencast from 02-05-2025 01:17:42 PM.mp4"  # Change this to your video file
output_gif = "/home/peeradon/Videos/Screencasts/wheel_test.gif"
video_to_gif(video_path, output_gif, fps=10, speed=0.5) 
