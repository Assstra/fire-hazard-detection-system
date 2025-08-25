# infrastructure

To train my models I wanted to automate it using Kubernetes and launch the training jobs through a CI/CD pipeline.

Components:
- `nvidia/gpu-operator` to give access to GPU resources
- [MLflow](https://mlflow.org/docs/latest/ml/tracking/) for tracking experiments
  - Without any persistence, because for now, I don't care (Of course, in production, we should consider using a persistent backend like PostgreSQL)
- Storage: Persistent Volume to share the dataset with `cv-training` (I wanted to keep it simple, but we should consider NFS or S3 in the future)

## To start

1. Once you have your kubernetes cluster, install the NVIDIA GPU operator:

```bash
helmfile sync
```

I have put every components I use in the [`helmfile.yaml`](helmfile.yaml), so if you want, you can also easily install them with `helm`. 

If you have any issues see:
- [Installing the NVIDIA GPU Operator - NVIDIA Documentation](https://docs.nvidia.com/datacenter/cloud-native/gpu-operator/latest/getting-started.html)
- [Deploy NVIDIA operator - Advanced Options and Configuration - RKE2 Documentation](https://docs.rke2.io/advanced#deploy-nvidia-operator): even if you don't have an RKE2 cluster, the troubleshooting could be useful

If you want a simple resource to test if the GPU scheduling is working, you can apply the following pod definition:

```bash
kubectl apply -f test-gpu.yaml
```

2. Create a namespace for the training jobs

```bash
kubectl create namespace cv-training
```

3. Create the persistent volume & claim

```bash
kubectl apply -f cv-training/pv.yaml
```

4. Create the training job

```bash
kubectl apply -f cv-training/job.yaml
```
