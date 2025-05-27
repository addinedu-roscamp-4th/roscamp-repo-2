// 다국어 번역 리소스
const translations = {
  // 한국어 번역
  ko: {
    // 카테고리
    categories: {
      recommended: '추천',
      food: '음식',
      beverage: '음료'
    },
    
    // 사이드바 버튼
    sidebar: {
      orderStatus: '주문 현황',
      callStaff: '직원 호출',
      callStaffInProgress: '요청 중',
      changeLanguage: '언어 변경'
    },
    
    // 직원 호출 모달
    staffCall: {
      title: '직원 호출 유형',
      close: '닫기',
      options: {
        help: '일반 도움요청',
        menu: '메뉴 문의',
        payment: '결제 도움',
        birthday: '생일 축하',
        other: '기타 요청'
      },
      success: '{type} 요청이 전달되었습니다. 잠시만 기다려주세요.',
      error: '직원 호출 요청에 실패했습니다. 다시 시도해주세요.'
    },
    
    // 언어 선택 모달
    language: {
      title: '언어 선택',
      close: '닫기',
      korean: '한국어',
      english: '영어',
      japanese: '일본어',
      success: '언어가 {language}로 변경되었습니다.'
    },
    
    // 장바구니
    cart: {
      title: '장바구니',
      empty: '장바구니가 비어 있습니다',
      addItem: '장바구니에 담기',
      removeItem: '항목 제거',
      totalPrice: '총 금액',
      checkout: '결제하기'
    },
    
    // 주문 상태
    orderStatus: {
      title: '주문 상태',
      waiting: '대기중',
      cooking: '조리중',
      ready: '완료',
      cancel: '취소하기',
      orderNumber: '주문번호',
      remainingTime: '남은 시간',
      minutes: '분',
      fullOrderCancel: '전체 주문 취소',
      menuPreparationStatus: '메뉴 준비 상태',
      paymentDetails: '결제 내역',
      paymentMethod: '결제 수단',
      creditCard: '신용카드',
      cash: '현금',
      mobilePayment: '모바일 결제',
      totalAmount: '총 금액',
      paymentTime: '결제 시각',
      table: '테이블',
      customerId: '고객 ID',
      orderTime: '주문 시각',
      menuTotalCount: '메뉴 총 개수',
      items: '개',
      confirmCancelAll: '정말로 전체 주문을 취소하시겠습니까?\n이 작업은 되돌릴 수 없습니다.',
      confirmCancelItem: '선택한 메뉴를 취소하시겠습니까?\n이 작업은 되돌릴 수 없습니다.',
      refreshed: '주문 정보를 새로고침했습니다.',
      success: {
        cancelled: '주문이 성공적으로 취소되었습니다.',
        itemCancelled: '선택한 메뉴가 취소되었습니다.'
      },
      error: {
        cancel: '주문 취소 중 오류 발생'
      }
    },
    
    // 결제 관련
    payment: {
      title: '결제',
      method: '결제 방법',
      card: '카드 결제',
      cash: '현금 결제',
      mobilePayment: '모바일 결제',
      customerCount: '인원 수',
      persons: '명',
      totalAmount: '총 결제금액',
      proceed: '결제 진행',
      cancel: '취소',
      success: '결제가 완료되었습니다',
      error: '결제 중 오류가 발생했습니다'
    },
    
    // 메뉴 아이템
    menu: {
      cookingTime: '예상 조리시간',
      minutes: '분',
      noDescription: '설명이 없습니다',
      price: '가격',
      quantity: '수량',
      addToCart: '장바구니에 담기',
      removeFromCart: '장바구니에서 제거',
      increaseQuantity: '수량 증가',
      decreaseQuantity: '수량 감소',
      viewDetails: '메뉴 상세 보기'
    },
    
    // 메뉴 데이터
    menuData: {
      "샐러드": {
        name: "샐러드",
        description: "싱싱한 채소를 버무려 만든 샐러드"
      },
      "스테이크": {
        name: "스테이크",
        description: "최고급 고기를 익힌 부드러운 스테이크"
      },
      "파스타": {
        name: "파스타",
        description: "현지인이 직접 만든 전통 양식 파스타"
      },
      "주스": {
        name: "주스",
        description: "상큼하고 달달한 과일 주스"
      },
      "와인": {
        name: "와인",
        description: "어디에나 어울리는 달달한 와인"
      }
    },
    
    // 공통 메시지
    common: {
      loading: '로딩 중...',
      error: '오류가 발생했습니다',
      retry: '다시 시도',
      close: '닫기',
      confirm: '확인',
      cancel: '취소',
      noMenuItemsInCategory: '이 카테고리에 메뉴가 없습니다',
      noMenuInfo: '메뉴 정보 없음',
      noAvailableMenuInfo: '현재 사용 가능한 메뉴 정보가 없습니다.',
      refresh: '다시 불러오기'
    }
  },
  
  // 영어 번역
  en: {
    // 카테고리
    categories: {
      recommended: 'Recommended',
      food: 'Food',
      beverage: 'Beverages'
    },
    
    // 사이드바 버튼
    sidebar: {
      orderStatus: 'Order Status',
      callStaff: 'Call Staff',
      callStaffInProgress: 'Calling...',
      changeLanguage: 'Change Language'
    },
    
    // 직원 호출 모달
    staffCall: {
      title: 'Staff Request Type',
      close: 'Close',
      options: {
        help: 'General Help',
        menu: 'Menu Inquiry',
        payment: 'Payment Help',
        birthday: 'Birthday Celebration',
        other: 'Other Request'
      },
      success: 'Your {type} request has been sent. Please wait.',
      error: 'Failed to call staff. Please try again.'
    },
    
    // 언어 선택 모달
    language: {
      title: 'Select Language',
      close: 'Close',
      korean: 'Korean',
      english: 'English',
      japanese: 'Japanese',
      success: 'Language changed to {language}'
    },
    
    // 장바구니
    cart: {
      title: 'Cart',
      empty: 'Your cart is empty',
      addItem: 'Add to Cart',
      removeItem: 'Remove Item',
      totalPrice: 'Total Price',
      checkout: 'Checkout'
    },
    
    // 주문 상태
    orderStatus: {
      title: 'Order Status',
      waiting: 'Waiting',
      cooking: 'Cooking',
      ready: 'Ready',
      cancel: 'Cancel',
      orderNumber: 'Order #',
      remainingTime: 'Remaining Time',
      minutes: 'min',
      fullOrderCancel: 'Cancel Entire Order',
      menuPreparationStatus: 'Menu Preparation Status',
      paymentDetails: 'Payment Details',
      paymentMethod: 'Payment Method',
      creditCard: 'Credit Card',
      cash: 'Cash',
      mobilePayment: 'Mobile Payment',
      totalAmount: 'Total Amount',
      paymentTime: 'Payment Time',
      table: 'Table',
      customerId: 'Customer ID',
      orderTime: 'Order Time',
      menuTotalCount: 'Total Menu Items',
      items: 'items',
      confirmCancelAll: 'Are you sure you want to cancel the entire order?\nThis action cannot be undone.',
      confirmCancelItem: 'Are you sure you want to cancel this menu item?\nThis action cannot be undone.',
      refreshed: 'Order information has been refreshed.',
      success: {
        cancelled: 'Order has been successfully cancelled.',
        itemCancelled: 'Selected menu item has been cancelled.'
      },
      error: {
        cancel: 'Error occurred while cancelling order'
      }
    },
    
    // 결제 관련
    payment: {
      title: 'Payment',
      method: 'Payment Method',
      card: 'Card Payment',
      cash: 'Cash Payment',
      mobilePayment: 'Mobile Payment',
      customerCount: 'Number of People',
      persons: 'persons',
      totalAmount: 'Total Amount',
      proceed: 'Proceed to Payment',
      cancel: 'Cancel',
      success: 'Payment Complete',
      error: 'Error during payment'
    },
    
    // 메뉴 아이템
    menu: {
      cookingTime: 'Estimated cooking time',
      minutes: 'min',
      noDescription: 'No description available',
      price: 'Price',
      quantity: 'Quantity',
      addToCart: 'Add to Cart',
      removeFromCart: 'Remove from Cart',
      increaseQuantity: 'Increase Quantity',
      decreaseQuantity: 'Decrease Quantity',
      viewDetails: 'View Details'
    },
    
    // 메뉴 데이터
    menuData: {
      "샐러드": {
        name: "Salad",
        description: "Fresh salad made with crisp vegetables"
      },
      "스테이크": {
        name: "Steak",
        description: "Tender steak made from premium quality meat"
      },
      "파스타": {
        name: "Pasta",
        description: "Traditional pasta made by local chef"
      },
      "주스": {
        name: "Juice",
        description: "Refreshing and sweet fruit juice"
      },
      "와인": {
        name: "Wine",
        description: "Sweet wine that goes well with any dish"
      }
    },
    
    // 공통 메시지
    common: {
      loading: 'Loading...',
      error: 'An error occurred',
      retry: 'Retry',
      close: 'Close',
      confirm: 'Confirm',
      cancel: 'Cancel',
      noMenuItemsInCategory: 'No menu items in this category',
      noMenuInfo: 'No Menu Information',
      noAvailableMenuInfo: 'No menu information is currently available.',
      refresh: 'Refresh'
    }
  },
  
  // 일본어 번역
  ja: {
    // 카테고리
    categories: {
      recommended: 'おすすめ',
      food: '食べ物',
      beverage: '飲み物'
    },
    
    // 사이드바 버튼
    sidebar: {
      orderStatus: '注文状況',
      callStaff: 'スタッフ呼出',
      callStaffInProgress: '呼出中',
      changeLanguage: '言語変更'
    },
    
    // 직원 호출 모달
    staffCall: {
      title: 'スタッフ呼出タイプ',
      close: '閉じる',
      options: {
        help: '一般的なサポート',
        menu: 'メニューの問い合わせ',
        payment: '支払いサポート',
        birthday: 'お誕生日のお祝い',
        other: 'その他のリクエスト'
      },
      success: '{type}リクエストが送信されました。少々お待ちください。',
      error: 'スタッフの呼出に失敗しました。もう一度お試しください。'
    },
    
    // 언어 선택 모달
    language: {
      title: '言語選択',
      close: '閉じる',
      korean: '韓国語',
      english: '英語',
      japanese: '日本語',
      success: '言語が{language}に変更されました'
    },
    
    // 장바구니
    cart: {
      title: 'カート',
      empty: 'カートは空です',
      addItem: 'カートに追加',
      removeItem: '商品を削除',
      totalPrice: '合計金額',
      checkout: '決済する'
    },
    
    // 주문 상태
    orderStatus: {
      title: '注文状況',
      waiting: '待機中',
      cooking: '調理中',
      ready: '完了',
      cancel: 'キャンセル',
      orderNumber: '注文番号',
      remainingTime: '残り時間',
      minutes: '分',
      fullOrderCancel: '注文全体のキャンセル',
      menuPreparationStatus: 'メニュー準備状況',
      paymentDetails: '決済内訳',
      paymentMethod: '決済方法',
      creditCard: 'クレジットカード',
      cash: '現金',
      mobilePayment: 'モバイル決済',
      totalAmount: '合計金額',
      paymentTime: '決済時間',
      table: 'テーブル',
      customerId: '顧客ID',
      orderTime: '注文時間',
      menuTotalCount: 'メニュー合計数',
      items: '個',
      confirmCancelAll: '全ての注文をキャンセルしてもよろしいですか？\nこの操作は元に戻せません。',
      confirmCancelItem: '選択したメニューをキャンセルしてもよろしいですか？\nこの操作は元に戻せません。',
      refreshed: '注文情報が更新されました。',
      success: {
        cancelled: '注文が正常にキャンセルされました。',
        itemCancelled: '選択したメニューがキャンセルされました。'
      },
      error: {
        cancel: '注文キャンセル中にエラーが発生しました'
      }
    },
    
    // 결제 관련
    payment: {
      title: '決済',
      method: '決済方法',
      card: 'カード決済',
      cash: '現金決済',
      mobilePayment: 'モバイル決済',
      customerCount: '人数',
      persons: '人',
      totalAmount: '合計金額',
      proceed: '決済を続ける',
      cancel: 'キャンセル',
      success: '決済が完了しました',
      error: '決済中にエラーが発生しました'
    },
    
    // 메뉴 아이템
    menu: {
      cookingTime: '予想調理時間',
      minutes: '分',
      noDescription: '説明がありません',
      price: '価格',
      quantity: '数量',
      addToCart: 'カートに追加',
      removeFromCart: 'カートから削除',
      increaseQuantity: '数量を増やす',
      decreaseQuantity: '数量を減らす',
      viewDetails: 'メニューの詳細'
    },
    
    // 메뉴 데이터
    menuData: {
      "샐러드": {
        name: "サラダ",
        description: "新鮮な野菜で作ったサラダ"
      },
      "스테이크": {
        name: "ステーキ",
        description: "最高級の肉で作った柔らかいステーキ"
      },
      "파스타": {
        name: "パスタ",
        description: "地元シェフが作った伝統的なパスタ"
      },
      "주스": {
        name: "ジュース",
        description: "さわやかで甘いフルーツジュース"
      },
      "와인": {
        name: "ワイン",
        description: "どの料理にも合う甘いワイン"
      }
    },
    
    // 공통 메시지
    common: {
      loading: '読み込み中...',
      error: 'エラーが発生しました',
      retry: 'リトライ',
      close: '閉じる',
      confirm: '確認',
      cancel: 'キャンセル',
      noMenuItemsInCategory: 'このカテゴリにはメニューがありません',
      noMenuInfo: 'メニュー情報なし',
      noAvailableMenuInfo: '現在利用可能なメニュー情報はありません。',
      refresh: '再読み込み'
    }
  }
};

export default translations; 