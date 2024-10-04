import csv
from googletrans import Translator

with open('sih.csv', 'r') as file:
    
    csv_reader = csv.reader(file)
    for row in csv_reader:
        translator= Translator()
        res=translator.translate(row,dest="en")
        print(res.text)
        
        
        #print(f"Column 1: {column1}, Column 2: {column2}")




#import pandas as pd
#f=pd.read_csv("")
 
