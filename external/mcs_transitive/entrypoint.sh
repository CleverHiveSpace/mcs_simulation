#!/bin/bash

# Install transitiverobotics
echo ============= TRANSITIVE SETUP =============
# Try to get credentials from environment variables first, then from secrets
if [ -n "$TRANSITIVE_ID" ] && [ -n "$TRANSITIVE_TOKEN" ]; then
  # Use environment variables if available
  curl -sf "https://install.transitiverobotics.com?id=${TRANSITIVE_ID}&token=${TRANSITIVE_TOKEN}&docker=true" | bash
else
  echo "Error: Transitive credentials not found in environment"
  exit 1
fi

# Bootstrap the Transitive agent
if [ ! -e "$HOME/.transitive/.installation_complete" ]; then
  mkdir -p "$HOME/.transitive"
  cp -r /transitive-preinstalled/. "$HOME/.transitive"
  rm -rf /transitive-preinstalled
fi
cd $HOME/.transitive # NOTE: prevents from ENOENT certs/client.crt error
bash $HOME/.transitive/start_agent.sh
