using System;
using UnityEngine;
using Random = UnityEngine.Random;

public class FluidSimulation : MonoBehaviour {

    /// <summary>物体X</summary>
    [SerializeField] private GameObject nsobjectOriginal = null;
    /// <summary>Xサイズ</summary>
    [SerializeField] private int WX = 12;
    /// <summary>Yサイズ</summary>
    [SerializeField] private int WY = 12;
    /// <summary>Zサイズ</summary>
    [SerializeField] private int WZ = 12;
    /// <summary>デルタT</summary>
    [SerializeField] private float delta_t = 0.2f;
    /// <summary>レイノルズ数。大きいほど粘性が少ない</summary>
    [SerializeField] private float Re = 1000000.0f;
    /// <summary>SOR法で使う加速係数</summary>
    [SerializeField] private float omega = 1.8f;
    /// <summary>浮遊粒子数。これは流体の上に流れている質量０の点(dot)</summary>
    [SerializeField] private int rys_num = 1024;
    /// <summary>最小サイズ</summary>
    [SerializeField] private float min_size = 0.1f;
    /// <summary>最大倍率</summary>
    [SerializeField] private float size_mul = 0.1f;
    /// <summary>外力X</summary>
    [SerializeField] private int setx = 6;
    /// <summary>外力Y</summary>
    [SerializeField] private int sety = 6;
    /// <summary>外力Y</summary>
    [SerializeField] private int setz = 6;
    /// <summary>外力ベクトル</summary>
    [SerializeField] private Vector3 power = new Vector3(0.99f, 0f, 0f);

    /// <summary>X速度</summary>
    private double[,,] vx;
    /// <summary>Y速度</summary>
    private double[,,] vy;
    /// <summary>Z速度</summary>
    private double[,,] vz;
    /// <summary>変化後X速度</summary>
    private double[,,] vx_after;
    /// <summary>変化後Y速度</summary>
    private double[,,] vy_after;
    /// <summary>変化後Z速度</summary>
    private double[,,] vz_after;
    /// <summary>発散値</summary>
    private double[,,] s;
    /// <summary>圧力</summary>
    private double[,,] p;
    /// <summary>変化後圧力</summary>
    private double[,,] p_after;
    /// <summary>粒子の座標</summary>
    private Vector3[] rys;
    /// <summary>可視化オブジェクト</summary>
    private GameObject[] nsobject;

    void Start() {

        vx = new double[WX + 1, WY, WZ];
        vy = new double[WX, WY + 1, WZ];
        vz = new double[WX, WY, WZ + 1];
        vx_after = new double[WX + 1, WY, WZ];
        vy_after = new double[WX, WY + 1, WZ];
        vz_after = new double[WX, WY, WZ + 1];
        s = new double[WX, WY, WZ];
        p = new double[WX, WY, WZ];
        p_after = new double[WX, WY, WZ];
        rys = new Vector3[rys_num];
        nsobject = new GameObject[rys_num];
        for (var i=0; i<rys_num; i++) {
            rys[i] = new Vector3(
                Random.value * (1.0f * (float)(WX - 2)) + 1.0f,
                Random.value * (1.0f * (float)(WY - 2)) + 1.0f,
                Random.value * (1.0f * (float)(WZ - 2)) + 1.0f
            );
            nsobject[i] = Instantiate(nsobjectOriginal);
            nsobject[i].transform.position = rys[i];
        }
        setx = setx % (WX - 1);
        sety = sety % (WY - 1);
        setz = setz % (WZ - 1);

    }

    private void Update() {

        adve(); // 移流（一時風上差分）

        viscosity(); // 粘性

        // 外力
        if (Input.GetKey(KeyCode.Space)) {
            vx[setx, sety, setz] = power.x;
            vy[setx, sety, setz] = power.y;
            vz[setx, sety, setz] = power.z;
        }

        // 外力
        Gairyoku(new Vector3(0, WY / 2.0f, 0f), Vector3.forward);

        set(); // 壁速度0に固定

        div(); // ダイバージェンス計算

        poisson(); // ポアソン方程式の項

        rhs(); // 修正

        view(); // 可視化

    }

    /// <summary>
    /// 外力を与える
    /// </summary>
    public void Gairyoku(Vector3 position, Vector3 force) {
        var sx = (int)Mathf.Clamp(position.x + WX * 0.5f, 0.0f, 1.0f * WX - 1.1f);
        var sy = (int)Mathf.Clamp(position.y, 1.0f, 1.0f * WX - 1.1f);
        var sz = (int)Mathf.Clamp(position.z + WZ * 0.5f, 0.0f, 1.0f * WZ - 1.1f);
        vx[sx, sy, sz] = force.x;
        vy[sx, sy, sz] = force.y;
        vz[sx, sy, sz] = force.z;
    }

    /// <summary>
    /// 移流（一時風上差分）
    /// </summary>
    private void adve() {

        //まずはvxの更新から
        for (var x = 1; x < WX - 1; x++) {
            for (var y = 1; y < WY - 1; y++) {
                for (var z = 1; z < WZ - 1; z++) {

                    var u = vx[x, y, z];
                    var v = (vy[x-1, y, z] + vy[x, y, z] + vy[z-1, y+1, z] + vy[x, y+1, z]) * 0.25;
                    var w = (vz[x-1, y, z] + vz[x, y, z] + vz[z-1, y, z+1] + vz[x, y, z+1]) * 0.25;

                    //( ｕ>=0かつｖ>=0かつ w >= 0の場合 )
                    if (u >= 0.0 && v >= 0.0 && w >= 0.0) {
                        vx_after[x, y, z] = vx[x, y, z] - u * (vx[x, y, z] - vx[x-1, y, z]) * delta_t - v * (vx[x, y, z] - vx[x, y-1, z]) * delta_t - w * (vx[x, y, z] - vx[x, y, z-1]) * delta_t;
                    }
                    //( ｕ<0かつｖ>=0かつ w >= 0の場合 )
                    if (u < 0.0 && v >= 0.0 && w >= 0.0) {
                        vx_after[x, y, z] = vx[x, y, z] - u * (vx[x+1, y, z] - vx[x, y, z]) * delta_t - v * (vx[x, y, z] - vx[x, y-1, z]) * delta_t - w * (vx[x, y, z] - vx[x, y, z-1]) * delta_t;
                    }
                    //( ｕ>=0かつｖ<0かつ w >= 0の場合 )
                    if (u >= 0.0 && v < 0.0 && w >= 0.0) {
                        vx_after[x, y, z] = vx[x, y, z] - u * (vx[x, y, z] - vx[x-1, y, z]) * delta_t - v * (vx[x, y+1, z] - vx[x, y, z]) * delta_t - w * (vx[x, y, z] - vx[x, y, z-1]) * delta_t;
                    }
                    //( ｕ<0かつｖ<0かつ w => 0の場合 )
                    if (u < 0.0 && v < 0.0 && w >= 0.0) {
                        vx_after[x, y, z] = vx[x, y, z] - u * (vx[x+1, y, z] - vx[x, y, z]) * delta_t - v * (vx[x, y+1, z] - vx[x, y, z]) * delta_t - w * (vx[x, y, z] - vx[x, y, z-1]) * delta_t;
                    }

                    //( ｕ>=0かつｖ>=0かつ w < 0の場合 )
                    if (u >= 0.0 && v >= 0.0 && w < 0.0) {
                        vx_after[x, y, z] = vx[x, y, z] - u * (vx[x, y, z] - vx[x-1, y, z]) * delta_t - v * (vx[x, y, z] - vx[x, y-1, z]) * delta_t - w * (vx[x, y, z+1] - vx[x, y, z]) * delta_t;
                    }
                    //( ｕ<0かつｖ>=0かつ w < 0の場合 )
                    if (u < 0.0 && v >= 0.0 && w < 0.0) {
                        vx_after[x, y, z] = vx[x, y, z] - u * (vx[x+1, y, z] - vx[x, y, z]) * delta_t - v * (vx[x, y, z] - vx[x, y-1, z]) * delta_t - w * (vx[x, y, z+1] - vx[x, y, z]) * delta_t;
                    }
                    //( ｕ>=0かつｖ<0かつ w < 0の場合 )
                    if (u >= 0.0 && v < 0.0 && w < 0.0) {
                        vx_after[x, y, z] = vx[x, y, z] - u * (vx[x, y, z] - vx[x-1, y, z]) * delta_t - v * (vx[x, y+1, z] - vx[x, y, z]) * delta_t - w * (vx[x, y, z+1] - vx[x, y, z]) * delta_t;
                    }
                    //( ｕ<0かつｖ<0かつ w < 0の場合 )
                    if (u < 0.0 && v < 0.0 && w < 0.0) {
                        vx_after[x, y, z] = vx[x, y, z] - u * (vx[x+1, y, z] - vx[x, y, z]) * delta_t - v * (vx[x, y+1, z] - vx[x, y, z]) * delta_t - w * (vx[x, y, z+1] - vx[x, y, z]) * delta_t;
                    }

                }
            }
        }
        //次にvyの更新
        for (var x = 1; x < WX - 1; x++) {
            for (var y = 1; y < WY - 1; y++) {
                for (var z = 1; z < WZ - 1; z++) {

                    var u = (vx[x, y - 1, z] + vx[x + 1, y - 1, z] + vx[x, y, z] + vx[x + 1, y, z]) * 0.25;
                    var v = vy[x, y, z];
                    var w = (vz[x, y - 1, z] + vz[x, y - 1, z + 1] + vz[x, y, z] + vz[x, y, z + 1]) * 0.25;

                    //( ｕ>=0かつｖ>=0かつ w >= 0の場合 )
                    if (u >= 0.0 && v >= 0.0 && w >= 0.0) {
                        vy_after[x, y, z] = vy[x, y, z] - u * (vy[x, y, z] - vy[x - 1, y, z]) * delta_t - v * (vy[x, y, z] - vy[x, y - 1, z]) * delta_t - w * (vy[x, y, z] - vy[x, y, z - 1]) * delta_t;
                    }
                    //( ｕ<0かつｖ>=0かつ w >= 0の場合 )
                    if (u < 0.0 && v >= 0.0 && w >= 0.0) {
                        vy_after[x, y, z] = vy[x, y, z] - u * (vy[x + 1, y, z] - vy[x, y, z]) * delta_t - v * (vy[x, y, z] - vy[x, y - 1, z]) * delta_t - w * (vy[x, y, z] - vy[x, y, z - 1]) * delta_t;
                    }
                    //( ｕ>=0かつｖ<0かつ w >= 0の場合 )
                    if (u >= 0.0 && v < 0.0 && w >= 0.0) {
                        vy_after[x, y, z] = vy[x, y, z] - u * (vy[x, y, z] - vy[x - 1, y, z]) * delta_t - v * (vy[x, y + 1, z] - vy[x, y, z]) * delta_t - w * (vy[x, y, z] - vy[x, y, z - 1]) * delta_t;
                    }
                    //( ｕ<0かつｖ<0かつ w => 0の場合 )
                    if (u < 0.0 && v < 0.0 && w >= 0.0) {
                        vy_after[x, y, z] = vy[x, y, z] - u * (vy[x + 1, y, z] - vy[x, y, z]) * delta_t - v * (vy[x, y + 1, z] - vy[x, y, z]) * delta_t - w * (vy[x, y, z] - vy[x, y, z - 1]) * delta_t;
                    }

                    //( ｕ>=0かつｖ>=0かつ w < 0の場合 )
                    if (u >= 0.0 && v >= 0.0 && w < 0.0) {
                        vy_after[x, y, z] = vy[x, y, z] - u * (vy[x, y, z] - vy[x - 1, y, z]) * delta_t - v * (vy[x, y, z] - vy[x, y - 1, z]) * delta_t - w * (vy[x, y, z + 1] - vy[x, y, z]) * delta_t;
                    }
                    //( ｕ<0かつｖ>=0かつ w < 0の場合 )
                    if (u < 0.0 && v >= 0.0 && w < 0.0) {
                        vy_after[x, y, z] = vy[x, y, z] - u * (vy[x + 1, y, z] - vy[x, y, z]) * delta_t - v * (vy[x, y, z] - vy[x, y - 1, z]) * delta_t - w * (vy[x, y, z + 1] - vy[x, y, z]) * delta_t;
                    }
                    //( ｕ>=0かつｖ<0かつ w < 0の場合 )
                    if (u >= 0.0 && v < 0.0 && w < 0.0) {
                        vy_after[x, y, z] = vy[x, y, z] - u * (vy[x, y, z] - vy[x - 1, y, z]) * delta_t - v * (vy[x, y + 1, z] - vy[x, y, z]) * delta_t - w * (vy[x, y, z + 1] - vy[x, y, z]) * delta_t;
                    }
                    //( ｕ<0かつｖ<0かつ w < 0の場合 )
                    if (u < 0.0 && v < 0.0 && w < 0.0) {
                        vy_after[x, y, z] = vy[x, y, z] - u * (vy[x + 1, y, z] - vy[x, y, z]) * delta_t - v * (vy[x, y + 1, z] - vy[x, y, z]) * delta_t - w * (vy[x, y, z + 1] - vy[x, y, z]) * delta_t;
                    }
                }
            }
        }

        //次にvzの更新
        for (var x = 1; x < WX - 1; x++) {
            for (var y = 1; y < WY - 1; y++) {
                for (var z = 1; z < WZ - 1; z++) {

                    var u = (vx[x, y, z - 1] + vx[x + 1, y, z - 1] + vx[x, y, z] + vx[x + 1, y, z]) * 0.25;
                    var v = (vy[x, y - 1, z] + vy[x, y - 1, z + 1] + vy[x, y, z] + vy[x, y, z + 1]) * 0.25;
                    var w = vz[x, y, z];

                    //( ｕ>=0かつｖ>=0かつ w >= 0の場合 )
                    if (u >= 0.0 && v >= 0.0 && w >= 0.0) {
                        vz_after[x, y, z] = vz[x, y, z] - u * (vz[x, y, z] - vz[x - 1, y, z]) * delta_t - v * (vz[x, y, z] - vz[x, y - 1, z]) * delta_t - w * (vz[x, y, z] - vz[x, y, z - 1]) * delta_t;
                    }
                    //( ｕ<0かつｖ>=0かつ w >= 0の場合 )
                    if (u < 0.0 && v >= 0.0 && w >= 0.0) {
                        vz_after[x, y, z] = vz[x, y, z] - u * (vz[x + 1, y, z] - vz[x, y, z]) * delta_t - v * (vz[x, y, z] - vz[x, y - 1, z]) * delta_t - w * (vz[x, y, z] - vz[x, y, z - 1]) * delta_t;
                    }
                    //( ｕ>=0かつｖ<0かつ w >= 0の場合 )
                    if (u >= 0.0 && v < 0.0 && w >= 0.0) {
                        vz_after[x, y, z] = vz[x, y, z] - u * (vz[x, y, z] - vz[x - 1, y, z]) * delta_t - v * (vz[x, y + 1, z] - vz[x, y, z]) * delta_t - w * (vz[x, y, z] - vz[x, y, z - 1]) * delta_t;
                    }
                    //( ｕ<0かつｖ<0かつ w => 0の場合 )
                    if (u < 0.0 && v < 0.0 && w >= 0.0) {
                        vz_after[x, y, z] = vz[x, y, z] - u * (vz[x + 1, y, z] - vz[x, y, z]) * delta_t - v * (vz[x, y + 1, z] - vz[x, y, z]) * delta_t - w * (vz[x, y, z] - vz[x, y, z - 1]) * delta_t;
                    }

                    //( ｕ>=0かつｖ>=0かつ w < 0の場合 )
                    if (u >= 0.0 && v >= 0.0 && w < 0.0) {
                        vz_after[x, y, z] = vz[x, y, z] - u * (vz[x, y, z] - vz[x - 1, y, z]) * delta_t - v * (vz[x, y, z] - vz[x, y - 1, z]) * delta_t - w * (vz[x, y, z + 1] - vz[x, y, z]) * delta_t;
                    }
                    //( ｕ<0かつｖ>=0かつ w < 0の場合 )
                    if (u < 0.0 && v >= 0.0 && w < 0.0) {
                        vz_after[x, y, z] = vz[x, y, z] - u * (vz[x + 1, y, z] - vz[x, y, z]) * delta_t - v * (vz[x, y, z] - vz[x, y - 1, z]) * delta_t - w * (vz[x, y, z + 1] - vz[x, y, z]) * delta_t;
                    }
                    //( ｕ>=0かつｖ<0かつ w < 0の場合 )
                    if (u >= 0.0 && v < 0.0 && w < 0.0) {
                        vz_after[x, y, z] = vz[x, y, z] - u * (vz[x, y, z] - vz[x - 1, y, z]) * delta_t - v * (vz[x, y + 1, z] - vz[x, y, z]) * delta_t - w * (vz[x, y, z + 1] - vz[x, y, z]) * delta_t;
                    }
                    //( ｕ<0かつｖ<0かつ w < 0の場合 )
                    if (u < 0.0 && v < 0.0 && w < 0.0) {
                        vz_after[x, y, z] = vz[x, y, z] - u * (vz[x + 1, y, z] - vz[x, y, z]) * delta_t - v * (vz[x, y + 1, z] - vz[x, y, z]) * delta_t - w * (vz[x, y, z + 1] - vz[x, y, z]) * delta_t;
                    }
                }
            }
        }
        Array.Copy(vx_after, vx, vx.Length);
        Array.Copy(vy_after, vy, vy.Length);
        Array.Copy(vz_after, vz, vz.Length);
    }

    /// <summary>
    /// 粘性
    /// </summary>
    void viscosity() {
        for (var x = 1; x < WX - 1; x++) {
            for (var y = 1; y < WY - 1; y++) {
                for (var z = 1; z < WZ - 1; z++) {
                    vx_after[x, y, z] = vx[x, y, z] - 1.0 / Re * (vx[x + 1, y, z] + vx[x, y + 1, z] + vx[x, y, z + 1] + vx[x - 1, y, z] + vx[x, y - 1, z] + vx[x, y, z - 1]) * delta_t;
                    vy_after[x, y, z] = vy[x, y, z] - 1.0 / Re * (vy[x + 1, y, z] + vy[x, y + 1, z] + vy[x, y, z + 1] + vy[x - 1, y, z] + vy[x, y - 1, z] + vy[x, y, z - 1]) * delta_t;
                    vz_after[x, y, z] = vz[x, y, z] - 1.0 / Re * (vz[x + 1, y, z] + vz[x, y + 1, z] + vz[x, y, z + 1] + vz[x - 1, y, z] + vz[x, y - 1, z] + vz[x, y, z - 1]) * delta_t;
                }
            }
        }
        Array.Copy(vx_after, vx, vx.Length);
        Array.Copy(vy_after, vy, vy.Length);
        Array.Copy(vz_after, vz, vz.Length);
    }

    /// <summary>
    /// 壁速度0に固定
    /// </summary>
    void set() {
        for (var x = 0; x < WX; x++) {
            for (var y = 0; y < WY; y++) {
                for (var z = 0; z < WZ; z++) {
                    if (x == 0 || x == (WX - 1) || y == 0 || y == (WY - 1) || z == 0 || z == (WZ - 1)) {
                        vx[x, y, z] = 0.0;
                        vx[x + 1, y, z] = 0.0;
                        vy[x, y, z] = 0.0;
                        vy[z, y + 1, z] = 0.0;
                        vz[x, y, z] = 0.0;
                        vz[x, y, z + 1] = 0.0;
                    }
                }
            }
        }
    }

    /// <summary>
    /// ダイバージェンス計算
    /// </summary>
    void div() {
        for (var x = 1; x < WX - 1; x++) {
            for (var y = 1; y < WY - 1; y++) {
                for (var z = 0; z < WZ - 1; z++) {
                    s[x, y, z] = (-vx[x, y, z] - vy[x, y, z] - vz[x, y, z] + vx[x + 1, y, z] + vy[x, y + 1, z] + vz[x, y, z + 1]) / delta_t;
                }
            }
        }
    }

    /// <summary>
    /// ポアソン方程式の項
    /// </summary>
    void poisson() {
        for (var c = 0; c < 10; c++) {
            for (var x = 1; x < WX - 1; x++) {
                for (var y = 1; y < WY - 1; y++) {
                    for (var z = 1; z < WZ - 1; z++) {
                        //もし壁なら、ijの圧力を代入
                        if (x == 1) p[x - 1, y, z] = p[x, y, z];
                        if (x == WX - 1) p[x + 1, y, z] = p[x, y, z];
                        if (y == 1) p[x, y - 1, z] = p[x, y, z];
                        if (y == WY - 1) p[x, y + 1, z] = p[x, y, z];
                        if (z == 1) p[x, y, z - 1] = p[x, y, z];
                        if (z == WZ - 1) p[x, y, z + 1] = p[x, y, z];
                        //ここがSOR
                        p[x, y, z] = (1.0 - omega) * p[x, y, z] + omega / 8.0 * (p[x - 1, y, z] + p[x + 1, y, z] + p[x, y - 1, z] + p[x, y + 1, z] + p[x, y, z - 1] + p[x, y, z + 1] - s[x, y, z]);
                    }
                }
            }
        }
    }

    /// <summary>
    /// 修正
    /// </summary>
    void rhs() {
        for (var x = 1; x < WX - 1; x++) {
            for (var y = 1; y < WY - 1; y++) {
                for (var z = 1; z < WZ - 1; z++) {
                    vx[x, y, z] -= (p[x, y, z] - p[x - 1, y, z]) * delta_t;
                    vy[z, y, z] -= (p[x, y, z] - p[x, y - 1, z]) * delta_t;
                    vz[x, y, z] -= (p[x, y, z] - p[x, y, z - 1]) * delta_t;
                }
            }
        }
    }

    private int counter = 0;

    /// <summary>
    /// 可視化
    /// </summary>
    void view() {

        for (var i = 0; i < 1; i++) {
            rys[counter] = new Vector3(
                UnityEngine.Random.value * (1.0f * (float)(WX - 2)) + 1.0f,
                UnityEngine.Random.value * (1.0f * (float)(WY - 2)) + 1.0f,
                UnityEngine.Random.value * (1.0f * (float)(WZ - 2)) + 1.0f
            );
            counter = (counter + 1) % rys_num;
        }
        
        for (var i=0; i<rys_num; i++) {
            var xx = (double)Mathf.Clamp(rys[i].x, 0.0f, 1.0f * WX - 1.1f);
            var yy = (double)Mathf.Clamp(rys[i].y, 0.0f, 1.0f * WY - 1.1f);
            var zz = (double)Mathf.Clamp(rys[i].z, 0.0f, 1.0f * WZ - 1.1f);
            var ixx = (int)xx;
            var iyy = (int)yy;
            var izz = (int)zz;
            var sxx = xx - ixx;
            var syy = yy - iyy;
            var szz = zz - izz;
            var im1 = (ixx + 1) % WX;
            var jm1 = (iyy + 1) % WY;
            var km1 = (izz + 1) % WZ;
            //  速度情報の線形補完。自分のいる座標での正確な速度を計算
            var xsp = (
                (((vx[ixx, iyy, izz] * (1.0 - sxx)) + (vx[im1, iyy, izz] * sxx)) * (1.0 - syy) + ((vx[ixx, jm1, izz] * (1.0 - sxx)) + (vx[im1, jm1, izz] * sxx)) * syy) * (1.0 - szz) +
                (((vx[ixx, iyy, km1] * (1.0 - sxx)) + (vx[im1, iyy, km1] * sxx)) * (1.0 - syy) + ((vx[ixx, jm1, km1] * (1.0 - sxx)) + (vx[im1, jm1, km1] * sxx)) * syy) * szz
            ) * delta_t;

            var ysp = (
                (((vy[ixx, iyy, izz] * (1.0 - sxx)) + (vy[im1, iyy, izz] * sxx)) * (1.0 - syy) + ((vy[ixx, jm1, izz] * (1.0 - sxx)) + (vy[im1, jm1, izz] * sxx)) * syy) * (1.0 - szz) +
                (((vy[ixx, iyy, km1] * (1.0 - sxx)) + (vy[im1, iyy, km1] * sxx)) * (1.0 - syy) + ((vy[ixx, jm1, km1] * (1.0 - sxx)) + (vy[im1, jm1, km1] * sxx)) * syy) * szz
            ) * delta_t;

            var zsp = (
                (((vz[ixx, iyy, izz] * (1.0 - sxx)) + (vz[im1, iyy, izz] * sxx)) * (1.0 - syy) + ((vz[ixx, jm1, izz] * (1.0 - sxx)) + (vz[im1, jm1, izz] * sxx)) * syy) * (1.0 - szz) +
                (((vz[ixx, iyy, km1] * (1.0 - sxx)) + (vz[im1, iyy, km1] * sxx)) * (1.0 - syy) + ((vz[ixx, jm1, km1] * (1.0 - sxx)) + (vz[im1, jm1, km1] * sxx)) * syy) * szz
            ) * delta_t;

            xx += xsp;
            yy += ysp;
            zz += zsp;

            if (
                (xx >= (1.0 * WX - 1.1)) ||
                (yy >= (1.0 * WY - 1.1)) ||
                (zz >= (1.0 * WZ - 1.1)) ||
                (xx < 1.1) ||
                (yy < 1.1) ||
                (zz < 1.1)
                ) {
                rys[i] = new Vector3(
                    Random.value * (1.0f * (float)(WX - 2)) + 1.0f,
                    Random.value * (1.0f * (float)(WY - 2)) + 1.0f,
                    Random.value * (1.0f * (float)(WZ - 2)) + 1.0f
                );
            } else {
                rys[i].x = (float)xx;
                rys[i].y = (float)yy;
                rys[i].z = (float)zz;
            }
            var speed = new Vector3((float)xsp, (float)ysp, (float)zsp).magnitude;
            nsobject[i].transform.position = new Vector3(rys[i].x - (WX * 0.5f), rys[i].y - (WY * 0.5f), rys[i].z - (WZ * 0.5f));
            nsobject[i].transform.localScale = Vector3.one * (min_size + speed * size_mul);
        }

    }
}