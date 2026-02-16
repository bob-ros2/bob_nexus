# Observability Template

This template provides a full monitoring stack based on Grafana, Loki, and Promtail (PLG Stack).

## Features
- **Loki**: Stores logs with high efficiency.
- **Promtail**: Automatically scrapes logs from all entities under `/entities/*/*/*.log`.
- **Grafana**: Pre-configured dashboarding on port `3000`.

## Labels
Logs are automatically labeled with:
- `category`: The entity category (e.g., assistant, master).
- `entity_name`: The name of the entity.
- `log_file`: The log filename (stdout.log or stderr.log).
- `container`: The Docker container name (for container logs).

## Usage
Spawn this entity via CLI:
```bash
./master/cli.sh spawn infrastructure observability observability
./master/cli.sh up observability
```
Then navigate to `http://your-ip:3000`.
