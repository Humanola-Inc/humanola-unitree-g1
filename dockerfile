# If your glibc is lower than 2.35, the humanola python library will not work, which is why we have this docker container that will bridge the gap for you.

FROM python:3.11-slim-trixie AS compile

RUN apt update && apt install -y \
  build-essential \
  cmake \
  git \
  wget \
  libglib2.0-0

# Install mabda
WORKDIR /opt
RUN wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh -O /tmp/miniforge.sh && \
  bash /tmp/miniforge.sh -b -p /opt/conda && \
  rm /tmp/miniforge.sh && \
  /opt/conda/bin/mamba install python=3.11 -y && \
  /opt/conda/bin/mamba clean -ya
RUN /opt/conda/bin/mamba install pinocchio -y;


# Compile CYCLONEDDS
WORKDIR /opt
RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
WORKDIR /opt/cyclonedds
RUN cmake -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds -Bbuild .
WORKDIR /opt/cyclonedds/build
RUN cmake --build . --target install --config Release

# Install unitree sdk2py
WORKDIR /opt
RUN git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
WORKDIR /opt/unitree_sdk2_python
ENV CYCLONEDDS_HOME="/opt/cyclonedds"
RUN pip install --compile .
RUN cp -r unitree_sdk2py/utils/lib /usr/local/lib/python3.11/site-packages/unitree_sdk2py/utils
RUN rm /usr/local/lib/python3.11/site-packages/unitree_sdk2py/__init__.py

# Install Inspire Hands
WORKDIR /opt
RUN git clone https://github.com/TechShare-inc/inspire_demos.git
WORKDIR /opt/inspire_demos
RUN pip install --compile .

WORKDIR /opt
COPY ./requirements.txt /opt/requirements.txt
RUN pip install -r /opt/requirements.txt

FROM python:3.11-slim-trixie AS final
RUN apt update && apt install -y \
  ffmpeg \
  libsm6 \
  libxext6 \
  libstdc++6 \
  ca-certificates
RUN apt clean

# COPY AND RUN
COPY --from=compile /opt/cyclonedds /opt/cyclonedds
COPY --from=compile /opt/conda/lib/. /usr/local/lib/.
COPY --from=compile /opt/conda/lib/python3.11/site-packages/. /usr/local/lib/python3.11/site-packages/
COPY --from=compile /usr/local/lib/python3.11/site-packages/. /usr/local/lib/python3.11/site-packages/
COPY --from=compile /usr/local/bin/. /usr/local/bin/
# Explicitly copy unitree SDK files that pip doesn't include
COPY --from=compile /opt/unitree_sdk2_python/unitree_sdk2py/utils/lib /usr/local/lib/python3.11/site-packages/unitree_sdk2py/utils/lib
COPY --from=compile /opt/unitree_sdk2_python/unitree_sdk2py/g1 /usr/local/lib/python3.11/site-packages/unitree_sdk2py/g1

WORKDIR /app
COPY ./src/. .
COPY ./URDF /app/URDF

ENV PYTHONPATH="/app"
ENV LD_LIBRARY_PATH="/usr/local/lib"

ENTRYPOINT ["python3", "main_ez.py"]