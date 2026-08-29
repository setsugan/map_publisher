# map_publisher

[![CI](https://github.com/setsugan/map_publisher/actions/workflows/ci.yml/badge.svg)](https://github.com/setsugan/map_publisher/actions/workflows/ci.yml)
[![Format](https://github.com/setsugan/map_publisher/actions/workflows/format.yml/badge.svg)](https://github.com/setsugan/map_publisher/actions/workflows/format.yml)

- Nav2用の2D占有格子地図（PGM + YAML）を読み込み、/mapトピックに送信します。
- ライフサイクルノードではないので、map_serverに比べて扱いやすいです。

## パラメーター

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| map_yaml_path | string |  | 地図YAMLファイルのパス |
| frame_id | string | map | フレームID |

## パブリッシャー

| トピック名 | メッセージ型 | 説明 |
| --- | --- | --- |
| map | nav_msgs/msg/OccupancyGrid | 2D占有格子地図 |
