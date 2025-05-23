from google import genai
from google.genai import types

client = genai.Client(api_key="Enter your API key here")

# 1️⃣ Upload your image once
red_cube_file = client.files.upload(file="output_isaac.png")

# 2️⃣ Call generate_content with your system prompt + the uploaded file
response = client.models.generate_content(
    model="gemini-2.0-flash",
    config=types.GenerateContentConfig(
        system_instruction="You are a High level planner of the robot.(JUST GIVE THE ACTION LIST) Based on the user's request, you will provide a detailed plan to achieve the goal. You will also provide just list of action needed to achieve the goal. The Types of actions are: 1. Turn 90 degrees left, 2. Turn 90 degrees right, 3. Move forward 1 m, 4. Move backward 1 m. The User will say color of block along with is location (x,y) and assume the robot is in (0,0) Facing the x-axis.(JUST GIVE THE ACTION LIST) (DONT NOT GIVE ANY EXPLANATION))) (NOTE EACH GRID IS 1m x 1m)"
    ),
    contents=[
      red_cube_file,                   # the uploaded image reference
      "How should I move to the green cube at (10,3)"  
    ]
)

print(response.text)
