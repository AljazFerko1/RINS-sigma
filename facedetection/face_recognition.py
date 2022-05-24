# %%

from glob import glob
import keras_vggface
from keras_vggface.vggface import VGGFace
from keras_vggface.utils import preprocess_input
from keras_vggface.utils import decode_predictions
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
from mtcnn.mtcnn import MTCNN
from scipy.spatial.distance import cosine
from scipy.stats import wasserstein_distance
# %%


def extract_face(img, required_size=(224, 224)):
    # load image from file
    #img = plt.imread(filename)
    # create the detector, using default weights
    detector = MTCNN()
    # detect faces in the image
    results = detector.detect_faces(img)
    # extract the bounding box from the first face
    x1, y1, width, height = results[0]['box']
    x2, y2 = x1 + width, y1 + height
    # extract the face
    face = img[y1:y2, x1:x2]
    # resize pixels to the model size
    image = Image.fromarray(face)
    image = image.resize(required_size)
    face_array = np.asarray(image)
    return face_array
# %%


def read_images(path):
    images = {}
    path = path + '/*'
    for file in glob(path):
        name = file.split("\\")[1].split(".")[0]
        images[name] = plt.imread(file)
    return images
# %%


def get_embeddings(faces):
    embeddings = {}
    model = VGGFace(model='resnet50', include_top=False,
                    input_shape=(224, 224, 3), pooling='avg')
    for name, face in faces.items():
        sample = np.asarray(face, 'float32')
        sample = preprocess_input(sample, version=2)
        sample = np.asarray([sample])
        embeddings[name] = model.predict(sample)
    """
    faces = np.asarray(list(faces.values()))
    samples = np.asarray(faces, 'float32')
    samples = preprocess_input(samples, version=2)
    embeddings = model.predict(samples)
    """
    return embeddings
# %%


def clossest_match(new_embeding, embeddings):
    # calculate distance between embeddings
    print(f"new embedding: {new_embeding.shape}")
    for name, embedding in embeddings.items():
        print(f"{name}: {embedding.shape}")
    distances = [(cosine(embeding, new_embeding), name)
                 for name, embeding in embeddings.items()]
    print(distances)
    return min(distances)[1]


# %%
if __name__ == "__main__":
    images = read_images("faces")
    faces = {name: extract_face(image) for name, image in images.items()}
    embeds = get_embeddings(faces)
    # %%
    new = {}
    img = plt.imread("test2.jpg")
    new_face = {}
    new_face["new"] = extract_face(img)
    new_emb = {}
    new_emb = get_embeddings(new_face)
    plt.imshow(img)
    # %%
    best = clossest_match(new_emb["new"], embeds)
    print(best)
    plt.imshow(images[best])
# %%
