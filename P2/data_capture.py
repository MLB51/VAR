from GUI import GUI
from HAL import HAL
import cv2
#import orjson 
import json
# Enter sequential code!
import pandas as pd
SET = 1
JSON_PATH = f'/data/dataset{SET}.json'

stop = 1600
V = 0
W = 0
def getKeys():
    with open(f"/data/keys{SET}.json","r") as file:
      return json.load(file)
      #return orjson.loads(file.read())
  
def move(i, key):
    global V, W
    if(key==''):
      if(i%20==0):
        print(f'iter {i}: {V} , {W}')
        
      return
    
    if key == 'w':
      V += 0.25
    elif key == 'a':
      W += 0.1
    elif key == 's':
      V -= 0.25
    elif key == 'd':
      W -= 0.1
    
    HAL.setV(V)
    HAL.setW(W)
    
    print(f'iter {i}: {V} , {W}')
    
def append_to_json(data, i):
    if(i==0):
        df = pd.DataFrame()
    else:
        df = pd.read_json(JSON_PATH)
      
    new_row = pd.DataFrame([data])
    df = pd.concat([df, new_row], ignore_index=True)
    df.to_json(JSON_PATH, orient='records')
    
    
i = 0
keys = getKeys()

while True:
    if(i<=stop):
        img = HAL.getImage()
        path = f'/data/imgs_{SET}/frame_{i}.jpg'
        cv2.imwrite(path, img)
        key = keys.get(str(i), '')
        move(i, key)
      
        append_to_json({
          'path':path,
          'V':V,
          'W':W
        },i)
        i+=1
    else:
        print('Acaba el proceso de captura')