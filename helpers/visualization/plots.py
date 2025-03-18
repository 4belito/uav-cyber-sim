

import plotly.graph_objects as go

def plot_3d_interactive(markers, title="title",expand=0.2,ground=0):
    # Create a list to store all waypoints
    data = []
    all_x, all_y, all_z = [], [], []
    for label, marker_set in markers.items():
        x, y, z = zip(*marker_set.get("pos"))  # Extract positions
        color = marker_set.get("color")  # Default color
        all_x.extend(x)
        all_y.extend(y)
        all_z.extend(z)
        # Add a scatter plot for each marker set
        trace = go.Scatter3d(
            x=x, y=y, z=z,
            mode="markers",
            marker=dict(size=6, color=color),
            name=label
        )
        data.append(trace)

    # Compute axis limits with scaling
    plot_limits=[(min(all_x), max(all_x)),(min(all_y), max(all_y)),(min(all_z), max(all_z))]
    ranges = [[m-expand[i]*(M-m),M+expand[i]*(M-m)]  for i,(m,M) in enumerate(plot_limits)]
    if ground is not None:
        ranges[2][0]=ground
    # Create figure with all markers
    fig = go.Figure(data=data)
    fig.update_layout(
        title=dict(
            text=title,
            x=0.5,  # Centers the title
            xanchor="center"),
        scene=dict(
            xaxis_title="x",
            yaxis_title="y",
            zaxis_title="z",
            xaxis=dict(range=ranges[0]),
            yaxis=dict(range=ranges[1]),
            zaxis=dict(range=ranges[2])),
        width=800,  # Adjust figure size
        height=600
    )

    fig.show()  # Display the interactive plot