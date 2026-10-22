# 真机算法镜像（arm64 / RK3568）
#
# 为什么必须 arm64：现场算法容器跑在 RK3568 上（aarch64），
# x86-64 镜像在设备上 `podman run` 会直接报 exec format error。
#
# 构建（在仓库根做 build context）：
#     docker buildx build --platform linux/arm64 \
#         -f deploy/vplc/Dockerfile -t aic-vision:arm64 --load .
#   Windows 上需要 Docker Desktop + buildx（QEMU 模拟，第一次会慢）；
#   在设备上直接 `podman build` 更快，见 README。
#
# 坑：不要 COPY 整个 src/ —— 里面对容器没用（ROS 节点、launch、world 文件），
#     体积大且 `__pycache__` 会带进去。只拷真正要跑的四份。

FROM --platform=linux/arm64 python:3.10-slim

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    LANG=C.UTF-8

# libgl1/libglib2.0-0：opencv-python-headless 仍会 dlopen 这两个，
# 缺了报 "ImportError: libGL.so.1: cannot open shared object file"。
RUN apt-get update \
    && apt-get install -y --no-install-recommends libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY deploy/vplc/requirements-arm64.txt /app/requirements-arm64.txt
# 国内网络默认 PyPI 会 502/超时（本机实测），走镜像源
RUN pip install --no-cache-dir -r /app/requirements-arm64.txt \
        -i https://pypi.tuna.tsinghua.edu.cn/simple

# RSC SDK 用仓库内置的 wheel，不在镜像里联网装（现场经常没外网）
COPY src/plugins/aic_plc_gateway/third_party/PyPlcnextRsc-0.1.12-py3-none-any.whl /app/
RUN pip install --no-cache-dir /app/PyPlcnextRsc-0.1.12-py3-none-any.whl

# 跑起来真正需要的四份 + 官方标签表
COPY src/vision/aic_vision_trad/scripts/plate_detector.py     /app/
COPY src/vision/aic_vision_trad/scripts/defect_classifier.py  /app/
COPY src/vision/aic_vision_trad/scripts/run_to_plc.py         /app/
COPY src/plugins/aic_plc_gateway/scripts/rsc_link.py          /app/
COPY src/plugins/aic_plc_gateway/config/cam_official.yaml     /app/config/cam_official.yaml

# 容器内视角的 vPLC 地址不是 192.168.3.99，而是桥接 IP（见 plcnext_rsc_setup.md §1）
ENV PLCNEXT_HOST=10.88.0.2 \
    PLCNEXT_PORT=41100 \
    PLCNEXT_USER=admin

# 默认只做「标签表自检」，不带参数不会连 PLC —— 防止误跑
ENTRYPOINT ["python", "/app/run_to_plc.py"]
CMD ["--map", "/app/config/cam_official.yaml", "--probe", "--dry-run"]
