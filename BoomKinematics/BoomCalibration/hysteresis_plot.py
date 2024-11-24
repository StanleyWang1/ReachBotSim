import pandas as pd
import matplotlib.pyplot as plt

# Load the first CSV file
file1 = 'BoomKinematics/BoomCalibration/extending.csv'
data1 = pd.read_csv(file1)
# Extending Fit Line
a1 = 0.004026/100
b1 = 12.97/100

# Load the second CSV file
file2 = 'BoomKinematics/BoomCalibration/retracting.csv'
data2 = pd.read_csv(file2)
# Retracting Fit Line
a2 = 0.00417/100
b2 = 18.79/100

# Ensure the column names match your CSV file structure
x_column1 = 'Ticks'  # Replace with the x-axis column name in file1
y_column1 = 'Length'  # Replace with the y-axis column name in file1
x_column2 = 'Ticks'  # Replace with the x-axis column name in file2
y_column2 = 'Length'  # Replace with the y-axis column name in file2

# Plot both datasets
plt.figure(figsize=(10, 6))
plt.scatter(data1[x_column1], data1[y_column1]/100, color='#4682B4', label='Boom Extending', alpha=0.7)
plt.scatter(data2[x_column2], data2[y_column2]/100, color='orange', label='Boom Retracting', alpha=0.7)

# Create x-values for the regression lines
x_range1 = data1[x_column1]
x_range2 = data2[x_column2]
# Calculate the y-values for the regression lines
y_fit1 = a1 * x_range1 + b1
y_fit2 = a2 * x_range2 + b2
# Plot regression lines
plt.plot(x_range1, y_fit1, color='#4682B4', linestyle=':', linewidth=1.5, alpha=0.5, label='Extending Fit Line')
plt.plot(x_range2, y_fit2, color='orange', linestyle=':', linewidth=1.5, alpha=0.5, label='Retracting Fit Line')

# Customize the plot
plt.xlabel('Motor [ticks]')  # Replace with your desired x-axis label
plt.ylabel('Length [m]')  # Replace with your desired y-axis label
plt.title('Boom Deployer Hysteresis')
plt.legend()
plt.grid(True)

plt.savefig('BoomKinematics/BoomCalibration/DeployerHysteresis.png', dpi=300, bbox_inches='tight')
plt.show()
