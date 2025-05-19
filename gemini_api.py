from google import genai
from google.genai import types

client = genai.Client(api_key="Enter your API key here")

response = client.models.generate_content(
    model="gemini-2.0-flash",
    # This instruction frames the model as an expert
    config=types.GenerateContentConfig(
        system_instruction="You are a High level planner of the robot.(JUST GIVE THE ACTION LIST) Based on the user's request, you will provide a detailed plan to achieve the goal. You will also provide just list of action needed to achieve the goal. The Types of actions are: 1. Turn 90 degrees left, 2. Turn 90 degrees right, 3. Move forward 1 m, 4. Move backward 1 m. The User will say color of block along with is location (x,y) and assume the robot is in (0,0) Facing the x-axis.(JUST GIVE THE ACTION LIST) (DONT NOT GIVE ANY EXPLANATION)))",
    ),
    # The user-facing prompt
    contents="How to move to green cube (5,5)."
)

print(response.text)
