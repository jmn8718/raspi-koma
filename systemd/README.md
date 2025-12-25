# systemd service

Configuration files for systemd services to run the scripts on the boot of the raspberry

**NOTE** modify the script to point to your path. Also it is using a python virtual env, so if you do not use it, then you have to modify it.

**NOTE** `pi_start` service will not restart if it is stopped successfully, so when `pi_shutdown` service stop it, it does not restart and display the messages again.

## Instructions

1 - Copy the services to `/etc/systemd/system/`
2 - Execute the commands

```sh
sudo systemctl daemon-reload

sudo systemctl start pi_start.service
sudo systemctl start pi_shutdown.service

sudo systemctl status pi_start.service
sudo systemctl status pi_shutdown.service
```