import numpy as np
import pandas as pd
import scipy.optimize as opt
import matplotlib.pyplot as plt

def second_order_inverse_model(x, A, B, C, D, E):
    return A / (x + B) + C / (x + D) ** 2 + E
def fit_second_order_inverse(x_data, y_data):
    params, _ = opt.curve_fit(second_order_inverse_model, x_data, y_data, p0=[3, 0, 3, 0, 0])
    return params

def first_order_inverse_model(x, A, B, C):
    return A / (x + B) + C

def fit_first_order_inverse(x_data, y_data):
    params, _ = opt.curve_fit(first_order_inverse_model, x_data, y_data, p0=[10, 0, 1])
    return params

df = pd.read_csv("../data/Micromouse_Sensor_Calibration - Tabellenblatt1.csv", skiprows=1)

df.columns = ["Index", "Abstand_cm", "Left", "Center", "Right"]
df = df.drop(columns=["Index"])

df["Abstand_cm"] = df["Abstand_cm"].astype(str).str.replace(",", ".").astype(float)
df["Left"] = pd.to_numeric(df["Left"])/1000
df["Center"] = pd.to_numeric(df["Center"])/1000
df["Right"] = pd.to_numeric(df["Right"])/1000

df = df[(df["Abstand_cm"] < 25)]

sensors = ["Left", "Center", "Right"]
fitted_params = {}
plt.figure(figsize=(8, 5))

# for sensor in sensors:
#     A, B, C, D, E = fit_second_order_inverse(df[sensor], df["Abstand_cm"])
#     fitted_params[sensor] = (A, B, C, D, E)
#     print(f"{sensor}: d = {A:.4f} / (V + {B:.4f}) + {C:.4f} / (V + {D:.4f})^2 + {E:.4f}")
    
#     plt.scatter(df[sensor], df["Abstand_cm"], label=f"{sensor} Data", alpha=0.6)
    
#     x_fit = np.linspace(min(df[sensor]), max(df[sensor]), 100)
#     y_fit = second_order_inverse_model(x_fit, A, B, C, D, E)
#     plt.plot(x_fit, y_fit, label=f"{sensor} Fit")

for sensor in sensors:
    A, B, C = fit_first_order_inverse(df[sensor], df["Abstand_cm"])
    fitted_params[sensor] = (A, B, C)
    print(f"{sensor}: d = {A:.4f} / (V + {B:.4f}) + {C:.4f}")
    
    plt.scatter(df[sensor], df["Abstand_cm"], label=f"{sensor} Data", alpha=0.6)
    
    x_fit = np.linspace(min(df[sensor]), max(df[sensor]), 100)
    y_fit = first_order_inverse_model(x_fit, A, B, C)
    plt.plot(x_fit, y_fit, label=f"{sensor} Fit")

plt.ylabel("Distance [cm]")
plt.xlabel("Voltage [mV]")
plt.title("Distance vs Sensor Readings")
plt.legend()
plt.grid(True)

plt.show()
