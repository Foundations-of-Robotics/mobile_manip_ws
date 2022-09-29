# The littlest JupyterHub setup
Refer to the instruction [here](http://tljh.jupyter.org/en/latest/install/custom-server.html)
```
sudo apt install python3 python3-dev git curl python3-pip
curl https://raw.githubusercontent.com/jupyterhub/the-littlest-jupyterhub/master/bootstrap/bootstrap.py | sudo -E python3 - --admin <admin-user-name>
```
Then copy the Public IP of the server, and try accessing http://<public-ip> from your browser. If everything went well, this should give you a JupyterHub login page. Keep in mind that the default installation provides only with Python3 support. Log as administrator to the JupyterHub browser interface and from the menu select new terminal.
```
python3 -m pip install ipykernel ipywidgets pyyaml filterpy ipyevents
python3 -m ipykernel install --user
jupyter nbextension enable --py widgetsnbextension
jupyter nbextension enable --py --sys-prefix ipyevents
```
You can then add manually all the users that can have access to the Jupyter Hub (or use the scripts provided in `scripts/users_mgt`)
```
sudo nano /opt/tljh/config/config.yaml
sudo tljh-config reload
```
