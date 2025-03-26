import pandas as pd
import matplotlib.pyplot as plt
import scipy.stats as stats
import numpy as np

df = pd.read_csv("../data/Micromouse_Sensor_Calibration - Tabellenblatt1.csv", skiprows=1)

df.columns = ["Index", "Abstand_cm", "Left", "Center", "Right"]

df = df.drop(columns=["Index"])

df["Abstand_cm"] = df["Abstand_cm"].astype(str).str.replace(",", ".").astype(float)
df["Left"] = pd.to_numeric(df["Left"]) 
df["Center"] = pd.to_numeric(df["Center"]) 
df["Right"] = pd.to_numeric(df["Right"])

plt.figure(figsize=(8, 5))
# plt.plot(df["Abstand_cm"], df["Left"], label="Left", alpha=0.6)
# plt.plot(df["Abstand_cm"], df["Center"], label="Center", alpha=0.6)
# plt.plot(df["Abstand_cm"], df["Right"], label="Right", alpha=0.6)

# plt.xlabel("Distance [cm]")
# plt.ylabel("Voltage [V]")
# plt.title("Sensor Readings vs Distance")

plt.plot(df["Left"],  df["Abstand_cm"], label="Left")
plt.plot(df["Center"], df["Abstand_cm"],  label="Center")
plt.plot(df["Right"], df["Abstand_cm"],  label="Right")

plt.ylabel("Distance [cm]")
plt.xlabel("Voltage [mV]")
plt.title("Distance vs Sensor Readings")
plt.legend()
plt.grid(True)

plt.show()
