import os
import yaml

def generate_yaml(filename):
    # Enter the contents of your YAML file specific to your cluster.
    job_template = f"""
apiVersion: batch/v1
kind: Job
metadata:
  name: shenron-data-conversion-job{number}
  namespace: <enter_your_namespace_here>
spec:
  template:
    metadata:
      labels:
        app: shenron-data-conversion-job{number}
    spec:
      hostname: shenron-data-conversion-job{number}
      restartPolicy: Never
      containers:
      - name: shenron-data-conversion-job{number}
        image: <enter_your_image_here>
        command:
        - /bin/sh
        - -c
        - "cd /radar-imaging-dataset/carla_garage_radar/Data_Collection_Scripts/Start_Carla_Job_Scripts && bash job{number}.sh"
        env:
        - name: TZ
          value: "UTC"
        - name: SIZEW
          value: "1920"
        - name: SIZEH
          value: "1200"
        - name: REFRESH
          value: "60"
        - name: DPI
          value: "96"
        - name: CDEPTH
          value: "24"
        - name: PASSWD
          valueFrom:
            secretKeyRef:
              name:
              key: 
        stdin: true
        tty: true
        ports:
        - name: http
          containerPort: 8080
          protocol: TCP
        resources:
          limits:
            memory: 50Gi
            cpu: "32"
            nvidia.com/gpu: 1
          requests:
            memory: 50Gi
            cpu: "32"
            nvidia.com/gpu: 1
        volumeMounts:
        - mountPath: /dev/shm
          name: dshm
        - mountPath: /pvcvol
          name: pvcvol
        - mountPath: /cache
          name: egl-cache-vol
      dnsPolicy: None
      dnsConfig:
        nameservers:
        - 
      volumes:
      - name: dshm
        emptyDir:
          medium: Memory
      - name: egl-cache-vol
        emptyDir: {{}}
      - name: pvcvol
        persistentVolumeClaim:
          claimName: pvcvol
      affinity:
        nodeAffinity:
          requiredDuringSchedulingIgnoredDuringExecution:
            nodeSelectorTerms:
            - matchExpressions:
              - key: nvidia.com/gpu.product
                operator: In
                values:
                - NVIDIA-A10
"""
    return job_template
    

if __name__ == "__main__":

    MAX_JOBS = 210
    for number in range(MAX_JOBS):
        job_content = generate_yaml(number)
        yaml_content = yaml.safe_load(job_content)
        os.makedirs("jobs", exist_ok = True)
        with open(f"jobs/job{number}_job.yaml", "w") as yaml_file:
            yaml.dump(yaml_content, yaml_file, default_flow_style=False)
