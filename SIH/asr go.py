import os
from pytube import YouTube
from moviepy.editor import VideoFileClip

# Replace with your YouTube video URL
youtube_url = "https://www.youtube.com/watch?v=gYNlJQ-dIuY"

# Replace with your desired download directory
download_path = "/path/to/download/directory"

try:
    # Download the YouTube video using pytube
    yt = YouTube(youtube_url)
    video_stream = yt.streams.get_highest_resolution()
    video_path = os.path.join(download_path, f"{yt.title}.mp4")
    video_stream.download(output_path=download_path)

    # Use moviepy to process the video
    video_clip = VideoFileClip(video_path)
    # You can perform further processing on the video if needed

    print("Video downloaded and processed successfully.")
    print("File path:", video_path)
except Exception as e:
    print(f"An error occurred while downloading the video: {e}")
    video_path = None

# You can use the video_path variable to access the downloaded video file.
