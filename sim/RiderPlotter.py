import pandas as pd
import plotly.graph_objs as go
from dash import Dash, dcc, html, Input, Output
import threading
import time
import os

FILE_PATH = "pose"  # CSV-style file (UTF-16)
df = pd.DataFrame()
lock = threading.Lock()


def file_reader():
    global df
    last_lines_count = 0
    while True:
        if os.path.exists(FILE_PATH):
            try:
                with open(FILE_PATH, "r", encoding="utf-16") as f:
                    lines = f.read().splitlines()

                if not lines:
                    time.sleep(0.5)
                    continue

                header = lines[0].split(",")
                data_lines = lines[1:]

                # Only process new lines
                new_lines = data_lines[-(len(data_lines) - last_lines_count):] if last_lines_count else data_lines
                last_lines_count = len(data_lines)

                rows = []
                for line in new_lines:
                    if line.strip():
                        parts = line.strip().split(",")
                        if len(parts) == len(header):
                            row = dict(zip(header, parts))
                            rows.append(row)

                if rows:
                    with lock:
                        new_df = pd.DataFrame(rows)
                        new_df = new_df.apply(pd.to_numeric, errors="ignore")
                        if df.empty:
                            df = new_df
                        else:
                            df = pd.concat([df, new_df], ignore_index=True)

            except Exception as e:
                print("[WARN] Failed to read file:", e)

        time.sleep(0.5)


# Start the thread as daemon so it never blocks exit
threading.Thread(target=file_reader, daemon=True).start()

# --- Dash app ---
app = Dash(__name__)
app.title = "Pose Plotter"

app.layout = html.Div(
    style={"backgroundColor": "#111", "color": "#eee", "padding": "20px"},
    children=[
        html.H1("Pose Plotter Dashboard"),
        dcc.Dropdown(
            id="signal-dropdown",
            multi=True,
            placeholder="Select signals",
        ),
        dcc.Graph(id="graph", style={"height": "80vh"}),
        dcc.Interval(id="interval", interval=500, n_intervals=0),
    ],
)

@app.callback(
    Output("signal-dropdown", "options"),
    Input("interval", "n_intervals")
)
def update_dropdown(_):
    with lock:
        local_df = df.copy()
    if local_df.empty:
        return []
    signals = [c for c in local_df.columns if c.lower() != "time"]
    return [{"label": s, "value": s} for s in signals]

@app.callback(
    Output("graph", "figure"),
    [Input("interval", "n_intervals"),
     Input("signal-dropdown", "value")]
)
def update_graph(_, signals):
    with lock:
        local_df = df.copy()
    fig = go.Figure()
    if not local_df.empty and signals:
        x_axis = local_df["time"] if "time" in local_df.columns else local_df.index
        for col in signals:
            if col in local_df.columns:
                fig.add_trace(go.Scatter(x=x_axis, y=local_df[col], mode="lines", name=col))
    fig.update_layout(
        template="plotly_dark",
        paper_bgcolor="#111",
        plot_bgcolor="#111",
        xaxis_title="Time",
        title="Pose Telemetry Live"
    )
    return fig

if __name__ == "__main__":
    app.run(debug=False)  # disable debug to avoid reloading conflicts
