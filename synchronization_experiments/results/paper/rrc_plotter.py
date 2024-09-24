import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns



# Load data
data = {
    'Agents': [10, 10, 10, 10, 10, 10, 100, 100, 100, 100, 100, 100, 200, 200, 200, 200, 200, 200, 350, 350, 350, 350, 350, 350],
    'Actions': [1, 1, 4, 4, 8, 8, 1, 1, 4, 4, 8, 8, 1, 1, 4, 4, 8, 8, 1, 1, 4, 4, 8, 8],
    'Filtering': ['Filtered', 'Not Filtered', 'Filtered', 'Not Filtered', 'Filtered', 'Not Filtered', 'Filtered', 'Not Filtered', 'Filtered', 'Not Filtered', 'Filtered', 'Not Filtered', 'Filtered', 'Not Filtered', 'Filtered', 'Not Filtered', 'Filtered', 'Not Filtered', 'Filtered', 'Not Filtered', 'Filtered', 'Not Filtered', 'Filtered', 'Not Filtered'],
    'Mean RRC': [2.91e-3, 4.49e-3, 4.94e-3, 5.09e-3, 6.31e-3, 6.46e-3, 2.15e-2, 2.48e-2, 2.49e-2, 2.85e-2, 2.80e-2, 3.01e-2, 4.64e-2, 5.15e-2, 4.82e-2, 4.67e-2, 5.28e-2, 5.93e-2, 8.82e-2, 9.53e-2, 8.83e-2, 1.15e-1, 1.14e-1, 1.49e-1],
    'Error RRC': [1.71e-4, 1.45e-4, 2.38e-4, 2.31e-4, 6.84e-4, 1.76e-4, 7.95e-4, 1.13e-3, 8.89e-4, 1.94e-3, 5.32e-4, 8.90e-4, 1.89e-3, 2.39e-3, 1.84e-3, 1.07e-2, 8.53e-4, 2.03e-3, 7.74e-3, 9.80e-3, 2.32e-3, 1.23e-2, 7.93e-3, 4.52e-3],
    'Mean MIP': [3.48e-5, 1.51e-3, 1.95e-3, 2.22e-3, 3.88e-3, 3.12e-3, 5.93e-5, 2.61e-3, 2.24e-3, 5.75e-3, 5.78e-3, 7.68e-3, 1.08e-4, 4.68e-3, 3.02e-3, 9.12e-3, 5.54e-3, 1.29e-2, 1.67e-4, 6.52e-3, 3.03e-3, 1.78e-2, 7.26e-3, 2.94e-2],
    'Error MIP': [1.14e-6, 2.13e-5, 1.02e-4, 1.66e-4, 4.76e-4, 4.91e-5, 8.34e-7, 1.47e-4, 1.23e-4, 7.84e-4, 2.24e-4, 3.70e-4, 1.24e-5, 1.80e-4, 3.14e-4, 1.74e-3, 1.24e-3, 1.14e-4, 2.20e-5, 6.99e-4, 1.51e-4, 6.51e-3, 3.40e-4, 5.44e-3]
}

df = pd.DataFrame(data)

# Create a new column combining Filtering and Actions
df['Filtering_Actions'] = df.apply(
    lambda row: f"{row['Filtering']} {row['Actions']} Action Requested" 
    if row['Actions'] == 1 
    else f"{row['Filtering']} {row['Actions']} Actions Requested", 
    axis=1
)

palette = sns.color_palette("tab10", n_colors=3)
# Plot
fig, axs = plt.subplots(2, 1, figsize=(12, 12))

# Plot for Mean RRC
sns.barplot(data=df, x='Agents', y='Mean RRC', hue='Filtering_Actions', ci=None, ax=axs[0], dodge=True)
#axs[0].set_yscale('log')
for i, bar in enumerate(axs[0].patches):
    x = bar.get_x() + bar.get_width() / 2
    y = bar.get_height()
    index = -1
    for j in range(24):
        if df.iloc[j]['Mean RRC'] == y:
            index = j
            break
    if 'Not Filtered' in df.iloc[index]['Filtering_Actions']:
        bar.set_hatch('////')
    if '1' in df.iloc[index]['Filtering_Actions']:
        bar.set_facecolor(palette[0])
    elif '4' in df.iloc[index]['Filtering_Actions']:
        bar.set_facecolor(palette[1])
    else:
        bar.set_facecolor(palette[2])
    error = df.iloc[index]['Error RRC']
    axs[0].errorbar(x, y, yerr=error, fmt='none', c='black', capsize=3)
axs[0].set_title('Mean RRC Time by Number of Agents, Filtering and Actions', fontsize=34, fontweight='bold', pad=15)
axs[0].set_ylabel('Time [s]', fontsize=30)
axs[0].set_xlabel('Number of Agents',fontsize=30)

handles, labels = axs[0].get_legend_handles_labels()
order = [0, 2, 4, 1, 3, 5]  # Adjust based on your legend entries
legend = axs[0].legend([handles[idx] for idx in order], 
                       [labels[idx] for idx in order], 
                       title='Filtering_Actions', fancybox=True, shadow=True, ncol=2, prop={'size': 32})
legend.set_title('Filtering and Actions Requested', prop={'size': 30, 'weight': 'bold'})

axs[0].tick_params(axis='x', labelsize=30)
axs[0].tick_params(axis='y', labelsize=30)
# Plot for Mean MIP
sns.barplot(data=df, x='Agents', y='Mean MIP', hue='Filtering_Actions', ci=None, ax=axs[1], dodge=True)
#axs[1].set_yscale('log')
for i, bar in enumerate(axs[1].patches):
    x = bar.get_x() + bar.get_width() / 2
    y = bar.get_height()
    index = -1
    for j in range(24):
        if df.iloc[j]['Mean MIP'] == y:
            index = j
            break
    
    if 'Not Filtered' in df.iloc[index]['Filtering_Actions']:
        bar.set_hatch('////')
    if '1' in df.iloc[index]['Filtering_Actions']:
        bar.set_facecolor(palette[0])
    elif '4' in df.iloc[index]['Filtering_Actions']:
        bar.set_facecolor(palette[1])
    else:
        bar.set_facecolor(palette[2])
    error = df.iloc[index]['Error MIP']
    axs[1].errorbar(x, y, yerr=error, fmt='none', c='black', capsize=3)
axs[1].set_title('Mean Confirmation Time by Number of Agents, Filtering and Actions', fontsize=34, fontweight='bold', pad=15)
axs[1].set_ylabel('Time [s]',fontsize=30)
axs[1].set_xlabel('Number of Agents',fontsize=30)
handles, labels = axs[0].get_legend_handles_labels()
order = [0, 2, 4, 1, 3, 5]  # Adjust based on your legend entries
legend = axs[1].legend([handles[idx] for idx in order], 
                       [labels[idx] for idx in order], 
                       title='Filtering_Actions', fancybox=True, shadow=True, ncol=2, prop={'size': 32})
legend.set_title('Filtering and Actions Requested', prop={'size': 30, 'weight': 'bold'})
axs[1].tick_params(axis='x', labelsize=30)
axs[1].tick_params(axis='y', labelsize=30)





plt.tight_layout()
plt.subplots_adjust(left=0.06, right=0.99)
plt.show()
