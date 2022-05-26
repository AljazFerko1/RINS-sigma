import torch
from torchvision import datasets, models, transforms
import os
import numpy
from PIL import Image
import matplotlib.pyplot as plt
import time
import numpy as np


def food_classification(image_file): 
    input_size = 224

    data_transforms = {
        'train': transforms.Compose([
            transforms.RandomResizedCrop(input_size),
            transforms.RandomHorizontalFlip(),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ]),
        'val': transforms.Compose([
            transforms.Resize(input_size),
            #transforms.CenterCrop(input_size),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ]),
    }

    class_dict = {0:'baklava',
                1:'pizza',
                2:'pomfri',
                3:'solata',
                4:'torta'}

    #images_path = 'FoodClassification/images'
    model_path = '/home/domen/ROS/src/task1/scripts/Rand_squeez_1.1_try4_52_0.956.pt'

    model = torch.load(model_path, map_location=torch.device('cpu'))
    model.eval()

    img_p = Image.open(image_file)
    # img_p.show()
    # time.sleep(10)

    img = data_transforms['val'](img_p).unsqueeze(0)
    pred = model(img)

    pred_np = pred.cpu().detach().numpy().squeeze()
    class_ind = np.argmax(pred_np)

    print(pred_np)
    
    print("The detected food is " + class_dict[class_ind])

    return class_dict[class_ind]