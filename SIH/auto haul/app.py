from flask import Flask,render_template
import firebase_admin
from firebase_admin import credentials,db
cred = credentials.Certificate("database.json")
firebase_admin.initialize_app(cred,{
   "databaseURL": "https://sih1336-default-rtdb.firebaseio.com"})

app = Flask('__main__')

ref = db.reference('/')
data = ref.get()

@app.route('/')
def index():
    return render_template('index.html')

app.run(debug=True)