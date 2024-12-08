from sklearn.model_selection import train_test_split
import pandas as pd
from sklearn.linear_model import LogisticRegression

file_path = '/kaggle/input/comp0214-data-set/alzheimers_navigation_data.csv'
data = pd.read_csv(file_path)

X = data.drop(columns=['result'])
y = data['result']

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=16)

logreg = LogisticRegression(random_state=16)

# fit the model with data
logreg.fit(X_train, y_train)

y_pred = logreg.predict(X_test)