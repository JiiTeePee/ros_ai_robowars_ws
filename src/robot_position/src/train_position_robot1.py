#! /usr/bin/python3 # ei ole ros-tiedosto, varoiksi

import numpy as np                              # numeerinen kirjasto
import pandas as pd                             # moduli mikä helpottaa datasettien pyörittelyä, pip3 install pandas --user
import tensorflow as tf                                  
from tensorflow import keras                    # keras , abstraktioverkko
from tensorflow.keras import layers
from tensorflow.python.keras.utils.generic_utils import validate_config             # layers tasot

train_dataset_path = 'robot1_positions.csv'      # mistä löytyy datasetit, pitää olla string
validation_dataset_path = 'robot1_positions.csv'    # oikeassa pitää olla eri nimellä !
test_dataset_path  = 'robot1_positions.csv'       # tarvitaan kaksis erillistä datasettiä

column_names = ['yaw_radians', 'us1','us2',          # pitää olla string
'us3', 'us4', 'us5', 'us6',
'ground_truth_x', 'ground_truth_y']

raw_train_dataset = pd.read_csv(train_dataset_path, names = column_names)   #ladataan datasetti

print(raw_train_dataset.shape)                          #printataa datasetin muoto, 
print(raw_train_dataset.head)                           # muutama rivi, jokiasessa dataa 9 columm, 184 riviä

train_dataset = raw_train_dataset.copy()                  # jos puutteita ongelmia, pandaksen kanssa pois, jätetään kopio
                                                        # ajaa alimmasta vscodessa, polku jos eri kansiossa, abs hyv' ~

# opetusdata ja data mitä halutaan opettaa samassa, labelit otetaan erikseen datasta

train_label_x = train_dataset.pop('ground_truth_x') # pop otetaan pois ja tallennetaan, 7 column
train_label_y = train_dataset.pop('ground_truth_y')

#labelit irrallaan , yhdistetään, uusi datasetti jossa 2 memberiä

train_labels = pd.concat([train_label_x, train_label_y], axis=1)
print(train_labels.head)

print(train_dataset.shape)                          #printataa , 7 column, 184 rows
print(train_dataset.head)  

# mitä pitää tehdä myös, tehtiin yhdelle datasetille,hyvään tarvitaan kolme datasettiä, train, vailidation, testi

raw_validation_dataset = pd.read_csv(validation_dataset_path, names = column_names) 

validation_dataset = raw_validation_dataset.copy() 
validation_label_x = validation_dataset.pop('ground_truth_x') # pop otetaan pois ja tallennetaan, 7 column
validation_label_y = validation_dataset.pop('ground_truth_y')

validation_labels = pd.concat([validation_label_x, validation_label_y], axis=1)
print(validation_labels.head)

# testidatasetti

raw_test_dataset = pd.read_csv(test_dataset_path, names = column_names) 

test_dataset = raw_test_dataset.copy() 
test_label_x = test_dataset.pop('ground_truth_x') # pop otetaan pois ja tallennetaan, 7 column
test_label_y = test_dataset.pop('ground_truth_y')

test_labels = pd.concat([test_label_x, test_label_y], axis=1)
print(test_labels.head)

# validointia käyetään epokkien aikana, testillä ajetaan ettei overfittaa

# kuusi eri datasettiä , train , train label, labeloitu oikeat tulokset mitä yritetään opettaa
# valoidointi ja testi myös kaksi, tässä kaikki mitä data tarvitsee, seuraavaksi rakennetaan neuroverkko model

# muutama täysin kytketty layer, neuroverkko model, yksinkertainen

def build_model():
    model = keras.Sequential([
        layers.Dense(64, activation="relu", input_shape=[len(train_dataset.keys())]), # 64 neuronia, laskennallisesti yksinkertaisin, miinuksella nolla, plussa lineaarinen
        layers.Dense(64, activation="relu"),  
        layers.Dense(2)                             # output layer
    ])
    #
    # määritetään , data, opetetaan kahta arvoa x ja y, labelit x:lle ja :y ;lle, ulostulo arvo layer pitää olla kaksi arvoa, tärkeä                             
    # input sahpe määrittelee mitä otetaan sissän # sisään tulo ja ulostulo oikean muotoisia

    optimizer = tf.keras.optimizers.RMSprop(0.001)          # gradient , descent, kehittyneempi, learning rate

    model.compile(loss = 'mse',                             # mse mean square error, toisen asteen error
    optimizer = optimizer,                                  # 
    metrics=['mae', 'mse'])                                 # mitataan, mae

    return model

model = build_model()
print(model.summary())                                      # näyttääkö oikeanlaiselta

# train osuus
# pieni epokki määrä, range monta kertaa määrä läpi ,
# joka kerta kun 10 epokkia on mennyt läpi, onko virhe testisetillä pinemoi vai suurempi kuin ennen
#mean vaerage error pienempi, tallennetaan model, katoo milloin testaamisessa on paras kohta

EPOCHS = 10
mae = 0 #mena average error

for x in range(50):
    history = model.fit(
        train_dataset, train_labels,
        epochs=EPOCHS, validation_data=(validation_dataset, validation_labels), verbose=1)
    last_mae = mae
    loss, mae, mse = model.evaluate(test_dataset,test_labels, verbose=2)
    print("Testing set Mean Abs error: {:5.2f} position".format(mae))

    if mae < last_mae:
        print("saving model...")
        model_save_path = "saved_model/robot1_model"+ str(x*10) + "epochs"
        model.save(model_save_path)

