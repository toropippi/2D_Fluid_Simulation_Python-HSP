# 2D_Fluid_Simulation_Python-HSP
ブログの流体力学のサンプルコード  

## cfd.py：単体実行可能  

![84fac621](https://user-images.githubusercontent.com/44022497/62557661-54542180-b8b2-11e9-8978-8755b09f9873.gif)  
  
## nabie01.hsp:単体実行可能  

実行にはHSPインストールが必要。  
HSPのインストールはこちらから(Windowsのみ) https://hsp.tv/idman/download.html  
クリックしたところの流速が変わる  
![hsp](https://user-images.githubusercontent.com/44022497/87177306-663a1200-c316-11ea-843f-eb4052428bc7.jpg)
  
## Unityフォルダ

UnityフォルダのほうはC#移植版となっている  
![mudai](https://user-images.githubusercontent.com/44022497/87177158-28d58480-c316-11ea-9429-a61eb6fe8161.jpg)

## さらに移流精度の良いやつ

上記3つは一次風上差分のコードとなっており、やや精度が悪い  
そこでCIP法を使って実装したコードを別Repositoryで公開しているのでそれも適宜参照を  
https://github.com/toropippi/Unity_GPGPU_2DFluid_CIPScheme
![image](https://user-images.githubusercontent.com/44022497/62557483-0b03d200-b8b2-11e9-93b1-140d2aee6101.png)
