
from teleoperation.app import app
from teleoperation.layout import layout as main_layout
app.layout = main_layout

if __name__ == "__main__":
    app.run_server(debug=True)
