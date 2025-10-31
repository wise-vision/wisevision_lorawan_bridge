FROM docker:cli AS docker-cli

FROM python:3.11-slim

COPY --from=docker-cli /usr/local/bin/docker /usr/local/bin/docker

WORKDIR /app

COPY chirpstack_autogen_keys/pyproject.toml ./

RUN pip install --no-cache-dir \
    chirpstack-api \
    grpcio \
    protobuf \
    docker

COPY chirpstack_autogen_keys/main.py .

VOLUME /workspace

CMD ["python", "main.py"]
