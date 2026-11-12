# walle-litter-bot

All the code is in [`walle-pilot/`](walle-pilot/README.md). The containers are defined in
`docker-compose.yml`; from this folder:

```bash
make up      # start the pilot + micro-ROS Agent containers
make shell   # open a shell in the pilot container (run again for more shells)
make down    # stop them
```

Run `make` to list the other shortcuts.
